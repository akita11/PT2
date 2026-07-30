#include "encoder.h"
#include "pins.h"
#include <avr/interrupt.h>

volatile int32_t encoderLeftCount = 0;
volatile int32_t encoderRightCount = 0;

// 2bit状態(prev<<2 | curr)から回転方向(-1/0/+1)を求める標準的な
// 4遷移(グレイコード)デコードテーブル。
static const int8_t QDEC_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

static volatile uint8_t leftState = 0;  // bit1=EL_A, bit0=EL_B
static volatile uint8_t rightState = 0; // bit1=ER_A, bit0=ER_B

static inline void updateLeftEncoder() {
  uint8_t a = (PINB & (1 << PINB0)) ? 1 : 0;
  uint8_t b = (PIND & (1 << PIND2)) ? 1 : 0;
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (leftState << 2) | newState;
  encoderLeftCount += QDEC_TABLE[idx];
  leftState = newState;
}

static inline void updateRightEncoder() {
  uint8_t a = (PIND & (1 << PIND3)) ? 1 : 0;
  uint8_t b = (PIND & (1 << PIND4)) ? 1 : 0;
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (rightState << 2) | newState;
  encoderRightCount += QDEC_TABLE[idx];
  rightState = newState;
}

ISR(PCINT0_vect) {
  updateLeftEncoder();
}

ISR(PCINT2_vect) {
  updateLeftEncoder();
  updateRightEncoder();
}

void encoderInit() {
  pinMode(PIN_EL_A, INPUT_PULLUP);
  pinMode(PIN_EL_B, INPUT_PULLUP);
  pinMode(PIN_ER_A, INPUT_PULLUP);
  pinMode(PIN_ER_B, INPUT_PULLUP);
  // エンコーダがプッシュプル出力タイプの場合は上記をINPUTに変更すること。

  noInterrupts();
  // 初期状態を割り込み有効化前に確定させ、起動直後の誤カウントを防ぐ。
  leftState = ((PINB & (1 << PINB0)) ? 1 : 0) << 1 | ((PIND & (1 << PIND2)) ? 1 : 0);
  rightState = ((PIND & (1 << PIND3)) ? 1 : 0) << 1 | ((PIND & (1 << PIND4)) ? 1 : 0);

  PCMSK0 |= (1 << PCINT0);                                // EL_A (PB0)
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20); // EL_B, ER_A, ER_B
  PCICR |= (1 << PCIE0) | (1 << PCIE2);
  interrupts();
}
