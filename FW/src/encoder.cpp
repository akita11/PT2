#include "encoder.h"
#include "pins.h"
#include <avr/interrupt.h>

volatile int32_t encoderLeftCount = 0;
volatile int32_t encoderRightCount = 0;

#ifdef ENCODER_SINGLE_PHASE

// A相の立ち上がりのみでカウントする(回転方向によらず増加のみ)。
//   左: EL_A=PD3 -> 外部割り込みINT1(立ち上がりエッジ)
//   右: ER_A=PB0 -> Timer1入力キャプチャICP1(立ち上がりエッジ)
// いずれもエッジ自体がハードウェアのフラグに保持されるため、他のISRで割り込みが
// 待たされても、次の立ち上がりまで(1周期以内)に処理されれば取りこぼさない。
// (PCINTはISR内でレベルを読むため、待たされている間にHigh->Lowと戻ると取りこぼす)
// Timer1は左モータのPWM(OC1A/OC1B, Arduino標準の8bit位相基準PWM)と共用するが、
// 入力キャプチャはこのモードでも使え、PWMには影響しない(ICR1の値は使わない)。

ISR(INT1_vect) {
  encoderLeftCount++;
}

ISR(TIMER1_CAPT_vect) {
  encoderRightCount++;
}

void encoderInit() {
  pinMode(PIN_EL_A, INPUT);
  pinMode(PIN_EL_B, INPUT);
  pinMode(PIN_ER_A, INPUT);
  pinMode(PIN_ER_B, INPUT);

  noInterrupts();
  EICRA |= (1 << ISC11) | (1 << ISC10); // INT1: 立ち上がりエッジ
  EIFR = (1 << INTF1);   // 設定変更時に立った可能性のあるフラグをクリア
  EIMSK |= (1 << INT1);

  // TCCR1BのWGM/CSビット(Arduinoが設定したPWM用)は保持し、ICES1のみ追加する
  TCCR1B |= (1 << ICES1); // ICP1: 立ち上がりエッジ
  TIFR1 = (1 << ICF1);
  TIMSK1 |= (1 << ICIE1);
  interrupts();
}

#else // 2相(4逓倍)デコード

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
  uint8_t a = (PIND & (1 << PIND3)) ? 1 : 0;
  uint8_t b = (PIND & (1 << PIND4)) ? 1 : 0;
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (leftState << 2) | newState;
  encoderLeftCount += QDEC_TABLE[idx];
  leftState = newState;
}

static inline void updateRightEncoder() {
  uint8_t a = (PINB & (1 << PINB0)) ? 1 : 0;
  uint8_t b = (PIND & (1 << PIND2)) ? 1 : 0;
  uint8_t newState = (a << 1) | b;
  uint8_t idx = (rightState << 2) | newState;
  //encoderRightCount += QDEC_TABLE[idx];
  // 右モータの回転方向を左モータと同じにするため、符号を反転させる
  encoderRightCount -= QDEC_TABLE[idx];
  rightState = newState;
}

ISR(PCINT0_vect) {
  updateRightEncoder();
}

ISR(PCINT2_vect) {
  updateLeftEncoder();
  updateRightEncoder();
}

void encoderInit() {
  pinMode(PIN_EL_A, INPUT);
  pinMode(PIN_EL_B, INPUT);
  pinMode(PIN_ER_A, INPUT);
  pinMode(PIN_ER_B, INPUT);
  // エンコーダがプッシュプル出力タイプの場合は上記をINPUTに変更すること。

  noInterrupts();
  // 初期状態を割り込み有効化前に確定させ、起動直後の誤カウントを防ぐ。
  leftState = ((PIND & (1 << PIND3)) ? 1 : 0) << 1 | ((PIND & (1 << PIND4)) ? 1 : 0);
  rightState = ((PINB & (1 << PINB0)) ? 1 : 0) << 1 | ((PIND & (1 << PIND2)) ? 1 : 0);

  PCMSK0 |= (1 << PCINT0);                                // ER_A (PB0)
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20); // ER_B, EL_A, EL_B
  PCICR |= (1 << PCIE0) | (1 << PCIE2);
  interrupts();
}

#endif // ENCODER_SINGLE_PHASE
