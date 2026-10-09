#include "motion_control.h"
#include "encoder.h"
#include "motor_control.h"
#include <avr/interrupt.h>

// 制御周期。Timer2: 8MHz/64 = 125kHz(8us/カウント)、OCR2A+1カウントで1周期。
static constexpr uint8_t CTRL_PERIOD_MS = 2;
static constexpr uint8_t CTRL_OCR2A = CTRL_PERIOD_MS * 125 - 1; // 2ms -> 249

// PIDは固定小数点(Q16: 1.0 = 65536)の整数演算で行う(ISR内で浮動小数点を使わない)。
// 出力(PWM) = (KP*e + KI*Σe + KD*Δe) >> PID_SHIFT
//   e : 目標との誤差[パルス], Σe : 制御周期ごとの誤差の積算, Δe : 1周期での誤差の変化
// 各ゲインは1周期あたりの値なので、制御周期を変えたらKI/KDも見直すこと。
static constexpr uint8_t PID_SHIFT = 16;

struct AxisGain {
  int32_t kp;
  int32_t ki;
  int32_t kd;
  // 目標付近ではKp*errorだけではモータの静止摩擦(デッドゾーン)を超える出力が
  // 出せず、目標手前で完全に停止してしまう(Ki頼みだと収束に数十秒かかる)。
  // 移動中は出力の絶対値がこの値を下回らないようにして停止を防ぐ。
  // この値が実際の静止摩擦を超えられないと、「出力は下駄(この値)で貼り付いた
  // まま動かず、Ki(積分)が時間をかけて育ってようやく動き出す」という、
  // 目標手前で止まってなかなか到達しない症状になる。実測して調整すること。
  int16_t minKick;
  // 減速: 目標の手前decelDistパルスから、出力上限をmaxSpeedからapproachSpeedまで
  // 直線的に下げる(目標到達時にapproachSpeed)。到達したらブレーキをかける。
  // 移動中は動摩擦だけなので、approachSpeedはminKick(静止摩擦)より小さくてもよい
  // (減速区間では下駄もこの上限以下に抑える)。
  int16_t decelDist;
  int16_t approachSpeed;
  // ENCODER_SINGLE_PHASE時: ブレーキ指令後に惰性で進むパルス数の初期値。
  // 目標のこのパルス数手前でブレーキをかける(戻しの制御ができないため)。
  // MOTION_COAST_LEARNING定義時は、移動ごとの実測値で自動更新される。
  int16_t coastInit;
  // 以下はkp/kiから自動計算(int32のオーバーフロー防止用)。
  // 出力上限(255)を超える分のP項・積分は意味がないため、この範囲に制限する。
  int32_t pErrorLimit;
  int32_t integralLimit;
};

static constexpr AxisGain makeGain(int32_t kp, int32_t ki, int32_t kd, int16_t minKick,
                                   int16_t decelDist, int16_t approachSpeed, int16_t coastInit) {
  return {kp, ki, kd, minKick, decelDist, approachSpeed, coastInit,
          (255L << PID_SHIFT) / kp + 1,
          ki > 0 ? (255L << PID_SHIFT) / ki : 0};
}

// 左右のゲイン。モータ・ギヤの個体差に合わせて左右別々に調整する。
//   kp: 39322 = 0.6, ki: 39 = 0.0006(Ki=0.3[1/s]×2ms), kd: 983040 = 15.0(Kd=0.03[s]÷2ms)
// Kdは1周期ごとの生のパルス差分(整数の量子化ノイズ)を微分するため、大きくすると
// ノイズをそのまま出力に増幅してしまう。抑え気味にする。
// coastInitは減速+ブレーキで惰性が小さくなる想定の仮の値(学習で補正される)。
// (参考: 減速なし・コースト停止では、左は約95パルス、右はほぼ0パルス惰性で進んだ)
//                                        kp     ki  kd      minKick decelDist approachSpeed coastInit
static constexpr AxisGain GAIN_L = makeGain(39322, 39, 983040, 90,     300,      60,           5);
static constexpr AxisGain GAIN_R = makeGain(39322, 39, 983040, 90,     300,      60,           0);

// ブレーキ指令後、この時間カウントが変化しなければ停止(惰性終了)とみなす。
// ブレーキは次の移動指令(またはsetMotorXxx())まで保持する。
static constexpr uint8_t SETTLE_TICKS = 100 / CTRL_PERIOD_MS; // 100ms
// 惰性補正量の学習: 1回の移動の行き過ぎ量(実測-目標)の1/2^COAST_LEARN_SHIFTずつ補正する
static constexpr uint8_t COAST_LEARN_SHIFT = 1;
static constexpr int16_t COAST_MAX = 500;

enum : uint8_t {
  AXIS_IDLE,     // 停止中
  AXIS_MOVING,   // PID制御中
  AXIS_SETTLING, // ブレーキ指令後、惰性が収まるのを待っている
};

struct MotionAxis {
  volatile uint8_t state;
  int32_t targetCount;
  int8_t dir;       // ENCODER_SINGLE_PHASE時の回転方向(+1/-1)。カウントは常に増加のみ
  int32_t decelSlope; // 減速区間の出力上限の傾き[PWM/パルス] (Q16)。moveByPulses()で計算
  int32_t integral; // 誤差の積算Σe
  int32_t prevError;
  int16_t coast;    // 停止指令を出す目標手前のパルス数(ENCODER_SINGLE_PHASE時)
  int32_t stopCount;    // 停止指令を出した時点のカウント
  int32_t settleCount;  // 静止判定用の前回カウント
  uint8_t settleTicks;
  int32_t lastOvershoot;  // 前回の移動の行き過ぎ量(最終カウント-目標)。チューニング確認用
  int32_t lastError;  // チューニング確認用
  int16_t lastOutput; // チューニング確認用 (下駄/飽和適用後の実際の出力)
};

static volatile MotionAxis axisL = {AXIS_IDLE, 0, 1, 0, 0, 0, GAIN_L.coastInit, 0, 0, 0, 0, 0, 0};
static volatile MotionAxis axisR = {AXIS_IDLE, 0, 1, 0, 0, 0, GAIN_R.coastInit, 0, 0, 0, 0, 0, 0};

static volatile uint8_t gMaxSpeed = 0;
static volatile int32_t gTolerance = 0;

// 停止指令後、カウントが変化しなくなるまで待ち、行き過ぎ量を記録(学習)する
static void settleAxis(volatile MotionAxis &axis, int32_t count) {
  if (count != axis.settleCount) {
    axis.settleCount = count;
    axis.settleTicks = 0;
    return;
  }
  if (++axis.settleTicks < SETTLE_TICKS) {
    return;
  }
  axis.lastOvershoot = count - axis.targetCount;
#if defined(ENCODER_SINGLE_PHASE) && defined(MOTION_COAST_LEARNING)
  int32_t coast = axis.coast + (axis.lastOvershoot >> COAST_LEARN_SHIFT);
  axis.coast = (int16_t)constrain(coast, 0L, (int32_t)COAST_MAX);
#endif
  axis.state = AXIS_IDLE;
}

static void updateAxis(volatile MotionAxis &axis, const AxisGain &gain, volatile int32_t &countRef,
                       void (*setSpeed)(int16_t), void (*brake)()) {
  if (axis.state == AXIS_IDLE) {
    return;
  }
  int32_t count = countRef;
  if (axis.state == AXIS_SETTLING) {
    settleAxis(axis, count);
    return;
  }

  int32_t error = axis.targetCount - count;
  axis.lastError = error;

#ifdef ENCODER_SINGLE_PHASE
  // カウントは増加のみで行き過ぎを戻せないため、惰性で進む分だけ手前でブレーキを
  // かける(行き過ぎ(error<0)も完了扱い)
  if (error <= axis.coast) {
#else
  // 目標との差が許容誤差以下になった時点で制御完了とする
  if (labs(error) <= gTolerance) {
#endif
    brake();
    axis.lastOutput = 0;
    axis.stopCount = count;
    axis.settleCount = count;
    axis.settleTicks = 0;
    axis.state = AXIS_SETTLING;
    return;
  }

  int32_t derivative = error - axis.prevError;
  axis.prevError = error;

  int32_t newIntegral = constrain(axis.integral + error, -gain.integralLimit, gain.integralLimit);
  int32_t pError = constrain(error, -gain.pErrorLimit, gain.pErrorLimit);
  int32_t output = (gain.kp * pError + gain.ki * newIntegral + gain.kd * derivative) >> PID_SHIFT;

  // 減速: 目標手前decelDistパルスからは、出力上限を目標までの距離に比例して下げる
  int32_t limit = gMaxSpeed;
  int32_t dist = labs(error);
  if (dist < gain.decelDist) {
    int32_t decelLimit = gain.approachSpeed + ((axis.decelSlope * dist) >> PID_SHIFT);
    if (decelLimit < limit) {
      limit = decelLimit;
    }
  }
  // デッドゾーン対策: 移動中はモータが動く最低出力を保証する(減速中は上限以下に抑える)
  int32_t kick = (gain.minKick < limit) ? gain.minKick : limit;

#ifdef ENCODER_SINGLE_PHASE
  // 戻しの制御は行わない: 出力は常に進行方向(正)側に限定する
  if (output < kick) {
    output = kick;
  }
#else
  if (output >= 0 && output < kick) {
    output = kick;
  } else if (output < 0 && output > -kick) {
    output = -kick;
  }
#endif

  if (output > limit) {
    output = limit;
  } else if (output < -limit) {
    output = -limit;
  } else {
    // 出力が飽和していないときだけ積分を更新する(アンチワインドアップ)
    axis.integral = newIntegral;
  }

#ifdef ENCODER_SINGLE_PHASE
  output *= axis.dir;
#endif
  axis.lastOutput = (int16_t)output;
  setSpeed((int16_t)output);
}

// ISR処理時間の最大値(Timer2のカウント値, 1カウント=8us)。チューニング確認用。
static volatile uint8_t isrMaxTicks = 0;

ISR(TIMER2_COMPA_vect) {
  updateAxis(axisL, GAIN_L, encoderLeftCount, setMotorLeft, brakeMotorLeft);
  updateAxis(axisR, GAIN_R, encoderRightCount, setMotorRight, brakeMotorRight);
  // CTCでコンペアマッチ時にTCNT2=0に戻るため、ISR末尾のTCNT2が
  // コンペアマッチからの経過時間になる(ISR開始までの遅延も含む)
  uint8_t t = TCNT2;
  if (t > isrMaxTicks) {
    isrMaxTicks = t;
  }
}

uint16_t motionIsrMaxMicros(bool reset) {
  noInterrupts();
  uint8_t t = isrMaxTicks;
  if (reset) {
    isrMaxTicks = 0;
  }
  interrupts();
  return (uint16_t)t * 8;
}

void motionControlInit() {
  axisL.state = AXIS_IDLE;
  axisR.state = AXIS_IDLE;

  noInterrupts();
  TCCR2A = (1 << WGM21);  // CTC (OCR2Aをtopとする)
  TCCR2B = (1 << CS22);   // prescaler 64
  OCR2A = CTRL_OCR2A;     // 8MHz/64/(249+1) = 500Hz -> 2ms周期
  TCNT2 = 0;
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

static void startAxis(volatile MotionAxis &axis, const AxisGain &gain, int32_t count, int32_t delta, uint8_t maxSpeed) {
  // 減速区間の傾き(ISR内での除算を避けるためここで計算しておく)
  int32_t span = (int32_t)maxSpeed - gain.approachSpeed;
  axis.decelSlope = (span > 0 && gain.decelDist > 0) ? (span << PID_SHIFT) / gain.decelDist : 0;
#ifdef ENCODER_SINGLE_PHASE
  // カウントは回転方向によらず増加するため、目標は移動量の絶対値で設定し、
  // 回転方向はdirとして別に保持する
  axis.dir = (delta < 0) ? -1 : 1;
  if (delta < 0) delta = -delta;
#endif
  axis.targetCount = count + delta;
  axis.integral = 0;
  axis.prevError = delta;
  axis.state = (delta != 0) ? AXIS_MOVING : AXIS_IDLE;
}

void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, int32_t tolerance) {
  noInterrupts();
  gMaxSpeed = maxSpeed;
  gTolerance = tolerance;
  startAxis(axisL, GAIN_L, encoderLeftCount, deltaLeft, maxSpeed);
  startAxis(axisR, GAIN_R, encoderRightCount, deltaRight, maxSpeed);
  bool activeL = (axisL.state != AXIS_IDLE);
  bool activeR = (axisR.state != AXIS_IDLE);
  interrupts();

  if (!activeL) {
    setMotorLeft(0);
  }
  if (!activeR) {
    setMotorRight(0);
  }
}

bool motionIsActive() {
  return axisL.state != AXIS_IDLE || axisR.state != AXIS_IDLE;
}

void motionWaitComplete() {
  unsigned long lastPrintMs = 0;
  while (motionIsActive()) {
    unsigned long nowMs = millis();
    /*
    // チューニング確認用: 停止中もerr/outputが見えるよう50ms間隔で表示する。
    // outputがminKick付近に貼り付いたままerrが変化しない場合、
    // 静止摩擦をminKickで超えられていない(Kiのワインドアップ待ち)ことを示す。
    if (nowMs - lastPrintMs >= 50) {
      lastPrintMs = nowMs;
      Serial.print(F("L err=")); Serial.print(axisL.lastError);
      Serial.print(F(" out=")); Serial.print(axisL.lastOutput);
      Serial.print(F(" | R err=")); Serial.print(axisR.lastError);
      Serial.print(F(" out=")); Serial.println(axisR.lastOutput);
    }
    */
  }
}

void motionStop() {
  noInterrupts();
  axisL.state = AXIS_IDLE;
  axisR.state = AXIS_IDLE;
  interrupts();
  brakeMotorLeft();
  brakeMotorRight();
}

static void printAxisResult(const __FlashStringHelper *name, volatile MotionAxis &axis) {
  noInterrupts();
  int32_t target = axis.targetCount;
  int32_t stopCount = axis.stopCount;
  int32_t overshoot = axis.lastOvershoot;
  int16_t coast = axis.coast;
  interrupts();
  Serial.print(name);
  Serial.print(F(" target=")); Serial.print(target);
  Serial.print(F(" stop=")); Serial.print(stopCount);
  Serial.print(F(" final=")); Serial.print(target + overshoot);
  Serial.print(F(" over=")); Serial.print(overshoot);
  Serial.print(F(" coast=")); Serial.println(coast);
}

void motionPrintResult() {
  printAxisResult(F("L"), axisL);
  printAxisResult(F("R"), axisR);
}
