#include "motion_control.h"
#include "encoder.h"
#include "motor_control.h"
#include <avr/interrupt.h>
#include <math.h>

// PIDゲイン。単位: 出力(PWM)/誤差(パルス)。実機の負荷・エンコーダ分解能に
// 合わせて要調整。
static constexpr float PID_KP = 0.6f;
static constexpr float PID_KI = 0.3f;
// Kdは1msごとの生のパルス差分(整数の量子化ノイズ)を微分するため、大きくすると
// ノイズをそのまま出力に増幅してしまう。抑え気味にする。
static constexpr float PID_KD = 0.03f;

static constexpr float DT = 0.001f; // Timer2割り込み周期[s] (1ms)

// 目標付近ではKp*errorだけではモータの静止摩擦(デッドゾーン)を超える出力が
// 出せず、目標手前で完全に停止してしまう(Ki頼みだと収束に数十秒かかる)。
// 許容誤差外の間は出力の絶対値がこの値を下回らないようにして停止を防ぐ。
static constexpr float MOTION_MIN_KICK = 60.0f;

struct MotionAxis {
  volatile bool active;
  int32_t targetCount;
  float integral;
  float prevError;
};

static volatile MotionAxis axisL = {false, 0, 0.0f, 0.0f};
static volatile MotionAxis axisR = {false, 0, 0.0f, 0.0f};

static volatile uint8_t gMaxSpeed = 0;
static volatile int32_t gTolerance = 0;

static void updateAxis(volatile MotionAxis &axis, volatile int32_t &count, void (*setSpeed)(int16_t)) {
  if (!axis.active) {
    return;
  }

  float error = (float)(axis.targetCount - count);

  // 目標との差が許容誤差以下になった時点で制御完了とする
  if (fabsf(error) <= (float)gTolerance) {
    setSpeed(0);
    axis.active = false;
    return;
  }

  float derivative = (error - axis.prevError) / DT;
  axis.prevError = error;

  float newIntegral = axis.integral + error * DT;
  float output = PID_KP * error + PID_KI * newIntegral + PID_KD * derivative;

  // デッドゾーン対策: 許容誤差外ならモータが動く最低出力を保証する
  if (output >= 0.0f && output < MOTION_MIN_KICK) {
    output = MOTION_MIN_KICK;
  } else if (output < 0.0f && output > -MOTION_MIN_KICK) {
    output = -MOTION_MIN_KICK;
  }

  float limit = (float)gMaxSpeed;
  if (output > limit) {
    output = limit;
  } else if (output < -limit) {
    output = -limit;
  } else {
    // 出力が飽和していないときだけ積分を更新する(アンチワインドアップ)
    axis.integral = newIntegral;
  }

  setSpeed((int16_t)output);
}

ISR(TIMER2_COMPA_vect) {
  updateAxis(axisL, encoderLeftCount, setMotorLeft);
  updateAxis(axisR, encoderRightCount, setMotorRight);
}

void motionControlInit() {
  axisL.active = false;
  axisR.active = false;

  noInterrupts();
  TCCR2A = (1 << WGM21);  // CTC (OCR2Aをtopとする)
  TCCR2B = (1 << CS22);   // prescaler 64
  OCR2A = 124;            // 8MHz/64/(124+1) = 1000Hz -> 1ms周期
  TCNT2 = 0;
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, int32_t tolerance) {
  noInterrupts();
  gMaxSpeed = maxSpeed;
  gTolerance = tolerance;

  axisL.targetCount = encoderLeftCount + deltaLeft;
  axisL.integral = 0.0f;
  axisL.prevError = (float)deltaLeft;
  axisL.active = (deltaLeft != 0);

  axisR.targetCount = encoderRightCount + deltaRight;
  axisR.integral = 0.0f;
  axisR.prevError = (float)deltaRight;
  axisR.active = (deltaRight != 0);
  interrupts();

  if (!axisL.active) {
    setMotorLeft(0);
  }
  if (!axisR.active) {
    setMotorRight(0);
  }
}

bool motionIsActive() {
  return axisL.active || axisR.active;
}

void motionWaitComplete() {
  while (motionIsActive()) {
    // 完了までポーリング待機
/*
    Serial.print(encoderLeftCount);
    Serial.print(F(" "));
    Serial.println(encoderRightCount);
*/
  }
}

void motionStop() {
  noInterrupts();
  axisL.active = false;
  axisR.active = false;
  interrupts();
  setMotorLeft(0);
  setMotorRight(0);
}
