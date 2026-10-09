#include "program_runner.h"
#include "resistor_matrix.h"
#include "motion_control.h"
#include "ws2812_control.h"
#include "sw.h"

static constexpr uint8_t MOTION_MAX_SPEED = 100; // 移動時の出力上限 (PWM 0-255)
static constexpr uint8_t LED_RED_LEVEL = 30;     // 実行中LEDの赤の明るさ

static constexpr uint8_t LOOP_ROW = 0;
static constexpr uint8_t OP_FIRST_ROW = 1;
static constexpr uint8_t N_OPS = 12;
static constexpr uint8_t N_LOOP_OPS = 4;

// SW押下時に読み取ったブロック配置。実行中は再スキャンしない。
static int8_t program[4][4];
static bool aborted;

static uint8_t ledIndexOf(uint8_t row, uint8_t col) {
  return row * 4 + col;
}

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

static void setLed(uint8_t ledIndex, bool on) {
  ledSetPixel(ledIndex, on ? LED_RED_LEVEL : 0, 0, 0);
  ledShow();
}

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

  // WS2812の更新(show)は割り込みを約0.5ms止めエンコーダの取りこぼし要因となるため、
  // 点灯/消灯を切り替える時だけ呼ぶ
  bool ledOn = true;
  setLed(ledIndex, ledOn);
  unsigned long lastToggleMs = millis();
  while (motionIsActive()) {
    if (checkAbort()) {
      break;
    }
    unsigned long nowMs = millis();
    if (nowMs - lastToggleMs >= LED_BLINK_CYCLE / 2) {
      lastToggleMs = nowMs;
      ledOn = !ledOn;
      setLed(ledIndex, ledOn);
    }
  }
  setLed(ledIndex, false);

  // 慣性による次の動作へのずれを防ぐため、動作間で一旦停止する
  unsigned long startMs = millis();
  while (millis() - startMs < WAIT_AMONG_OPERATION) {
    if (checkAbort()) {
      break;
    }
  }
}

// loop1-loop4を順に実行する。ブロックがない位置、または入れ子のloopで終了する。
static void runSubroutine() {
  for (uint8_t col = 0; col < N_LOOP_OPS && !aborted; col++) {
    int8_t op = program[LOOP_ROW][col];
    if (op == OP_NONE || op == OP_LOOP) {
      break;
    }
    executeBlock(op, ledIndexOf(LOOP_ROW, col));
  }
}

void programRun() {
  resistorMatrixScan();
  memcpy(program, rmClass, sizeof(program));
  aborted = false;

  ledSetAll(0, 0, 0);
  ledShow();
  Serial.println(F("Program start."));

  for (uint8_t i = 0; i < N_OPS && !aborted; i++) {
    uint8_t row = OP_FIRST_ROW + i / 4;
    uint8_t col = i % 4;
    int8_t op = program[row][col];
    if (op == OP_NONE) {
      break;
    }
    uint8_t ledIndex = ledIndexOf(row, col);
    if (op == OP_LOOP) {
      // サブルーチン実行中は呼び出し元のloopブロックを赤で点灯させておく
      Serial.print(F("LED")); Serial.print(ledIndex); Serial.println(F(": loop"));
      setLed(ledIndex, true);
      runSubroutine();
      setLed(ledIndex, false);
    } else {
      executeBlock(op, ledIndex);
    }
  }

  motionStop();
  ledSetAll(0, 0, 0);
  ledShow();
  Serial.println(F("Program end."));
}
