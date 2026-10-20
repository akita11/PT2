#include "program_runner.h"
#include "pins.h"
#include "resistor_matrix.h"
#include "motion_control.h"
#include "ws2812_control.h"
#include "sw.h"

static constexpr uint8_t MOTION_MAX_SPEED = 100; // 移動時の出力上限 (PWM 0-255)
static constexpr uint8_t LED_LEVEL = 30;         // LEDの明るさ

static constexpr unsigned long LED_STARTUP_STEP_MS = 50;  // 起動時に1個ずつ点灯させる間隔[ms]
static constexpr unsigned long LED_END_BLINK_CYCLE = 200; // 終了時の全LED青点滅の周期[ms]
static constexpr uint8_t LED_END_BLINK_COUNT = 3;         // 終了時の全LED青点滅の回数

// ブロック未配置の位置にブロックが置かれたと判定するのに必要な、同じ判定結果の連続スキャン回数
// (1スキャン約20ms。置く途中の接触不安定で誤判定しないため)
static constexpr uint8_t BLOCK_STABLE_SCANS = 5;

static constexpr uint8_t LOOP_FIRST_LED = 0; // loop1 (row=0, col=0)
static constexpr uint8_t N_LOOP_OPS = 4;
static constexpr uint8_t OP_FIRST_LED = 4;   // op1 (row=1, col=0)
static constexpr uint8_t N_OPS = 12;

// SW押下時に読み取ったブロック配置。ブロック未配置の位置に後から置かれた場合のみ更新する。
// LEDの番号(row*4+col)と同じ並びの1次元配列として扱う。
static int8_t program[16];
static bool aborted;

// 実行中にSWが押されたら移動を中断し、以降の実行をすべて取りやめる
static bool checkAbort() {
  if (!aborted && swRead()) {
    motionStop();
    aborted = true;
    Serial.println(F("Aborted."));
    while (swRead()) {
      delay(10);
    }
  }
  return aborted;
}

enum LedColor : uint8_t { LED_OFF, LED_RED, LED_GREEN, LED_BLUE };

static void setLed(uint8_t ledIndex, LedColor color) {
  ledSetPixel(ledIndex,
              color == LED_RED ? LED_LEVEL : 0,
              color == LED_GREEN ? LED_LEVEL : 0,
              color == LED_BLUE ? LED_LEVEL : 0);
  ledShow();
}

// 点滅中のLEDを、半周期ごとに点灯/消灯切り替えする
struct Blinker {
  uint8_t ledIndex;
  unsigned long halfCycleMs;
  bool on;
  unsigned long lastToggleMs;

  void start() {
    on = true;
    lastToggleMs = millis();
    setLed(ledIndex, LED_RED);
  }
  // WS2812の更新(show)は割り込みを約0.5ms止めエンコーダの取りこぼし要因となるため、
  // 点灯/消灯を切り替える時だけ呼ぶ
  void update() {
    unsigned long nowMs = millis();
    if (nowMs - lastToggleMs >= halfCycleMs) {
      lastToggleMs = nowMs;
      on = !on;
      setLed(ledIndex, on ? LED_RED : LED_OFF);
    }
  }
};

static void executeBlock(int8_t op, uint8_t ledIndex) {
  int32_t dL = 0;
  int32_t dR = 0;
  switch (op) {
  case OP_FORWARD:  dL =  COUNT_STEP_MOVE; dR =  COUNT_STEP_MOVE; break;
  case OP_BACKWARD: dL = -COUNT_STEP_MOVE; dR = -COUNT_STEP_MOVE; break;
  case OP_LEFT:     dL = -STEP_ROTATE;     dR =  STEP_ROTATE;     break;
  case OP_RIGHT:    dL =  STEP_ROTATE;     dR = -STEP_ROTATE;     break;
  default: return;
  }

  Serial.print(F("LED")); Serial.print(ledIndex);
  Serial.print(F(": ")); Serial.println(RM_R_CLASS_LABEL[op]);

  moveByPulses(dL, dR, MOTION_MAX_SPEED);

  Blinker blinker = {ledIndex, LED_BLINK_CYCLE / 2, true, 0};
  blinker.start();
  while (motionIsActive()) {
    if (checkAbort()) {
      break;
    }
    blinker.update();
  }
  setLed(ledIndex, LED_OFF);

  // 慣性による次の動作へのずれを防ぐため、動作間で一旦停止する
  unsigned long startMs = millis();
  while (millis() - startMs < WAIT_AMONG_OPERATION) {
    if (checkAbort()) {
      break;
    }
  }
}

// 実行順序の途中でブロックが置かれていない位置に来た場合、その位置のLEDを
// LED_BLINK_CYCLE_ERROR周期で点滅させながら、ブロックが置かれるまで待つ。
// ブロックが置かれたらLEDを点灯に変えてWAIT_AFTER_BLOCK_PLACEDだけ待ち、
// 置かれたブロックの種別を返す(SWで中断された場合はOP_NONE)。
static int8_t waitForBlock(uint8_t ledIndex) {
  Serial.print(F("LED")); Serial.print(ledIndex); Serial.println(F(": waiting for block"));

  Blinker blinker = {ledIndex, LED_BLINK_CYCLE_ERROR / 2, true, 0};
  blinker.start();
  int8_t candidate = OP_NONE;
  uint8_t stableCount = 0;
  while (!checkAbort()) {
    blinker.update();
    resistorMatrixScan();
    int8_t c = rmClass[ledIndex / 4][ledIndex % 4];
    if (c != OP_NONE && c == candidate) {
      if (++stableCount >= BLOCK_STABLE_SCANS) {
        break;
      }
    } else {
      candidate = c;
      stableCount = 0;
    }
  }
  if (!aborted) {
    setLed(ledIndex, LED_RED);
    unsigned long startMs = millis();
    while (millis() - startMs < WAIT_AFTER_BLOCK_PLACED) {
      if (checkAbort()) {
        break;
      }
    }
  }
  setLed(ledIndex, LED_OFF);

  if (aborted) {
    return OP_NONE;
  }
  program[ledIndex] = candidate;
  return candidate;
}

// firstLedから始まるn個のブロックを順に実行する。実行するのは最後に置かれた
// ブロックまでで、その途中のブロック未配置の位置ではwaitForBlock()で待つ。
// isSubroutine=trueはloop1-loop4の実行で、loopブロックの入れ子は無視する。
static void runSequence(uint8_t firstLed, uint8_t n, bool isSubroutine) {
  int8_t last = -1;
  for (uint8_t i = 0; i < n; i++) {
    if (program[firstLed + i] != OP_NONE) {
      last = i;
    }
  }

  for (int8_t i = 0; i <= last && !aborted; i++) {
    uint8_t ledIndex = firstLed + i;
    int8_t op = program[ledIndex];
    if (op == OP_NONE) {
      op = waitForBlock(ledIndex);
      if (aborted) {
        break;
      }
    }

    if (op != OP_LOOP) {
      executeBlock(op, ledIndex);
    } else if (isSubroutine) {
      Serial.print(F("LED")); Serial.print(ledIndex); Serial.println(F(": nested loop ignored"));
    } else {
      // loop1-loop4の実行中は、呼び出し元のloopブロックを緑で点灯させておく
      Serial.print(F("LED")); Serial.print(ledIndex); Serial.println(F(": loop"));
      setLed(ledIndex, LED_GREEN);
      runSequence(LOOP_FIRST_LED, N_LOOP_OPS, true);
      setLed(ledIndex, LED_OFF);
    }
  }
}

void programStartupLed() {
  randomSeed(analogRead(PIN_RM_MUX_SIG) ^ micros());
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    // 暗すぎて点灯がわからない色は引き直す
    uint8_t r, g, b;
    do {
      r = random(LED_LEVEL + 1);
      g = random(LED_LEVEL + 1);
      b = random(LED_LEVEL + 1);
    } while (r + g + b < LED_LEVEL);
    // 点灯するのはi番目の1個のみ(他は消灯)
    ledSetAll(0, 0, 0);
    ledSetPixel(i, r, g, b);
    ledShow();
    delay(LED_STARTUP_STEP_MS);
  }
  ledSetAll(0, 0, 0);
  ledShow();
}

static void showProgramEnd() {
  for (uint8_t n = 0; n < LED_END_BLINK_COUNT; n++) {
    ledSetAll(0, 0, LED_LEVEL);
    ledShow();
    delay(LED_END_BLINK_CYCLE / 2);
    ledSetAll(0, 0, 0);
    ledShow();
    delay(LED_END_BLINK_CYCLE / 2);
  }
}

void programRun() {
  resistorMatrixScan();
  memcpy(program, rmClass, sizeof(program));
  aborted = false;

  ledSetAll(0, 0, 0);
  ledShow();
  Serial.println(F("Program start."));

  runSequence(OP_FIRST_LED, N_OPS, false);

  motionStop();
  ledSetAll(0, 0, 0);
  ledShow();
  if (!aborted) {
    showProgramEnd();
  }
  Serial.println(F("Program end."));
}
