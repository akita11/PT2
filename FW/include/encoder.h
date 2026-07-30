#pragma once
// モータに連動する2相ロータリエンコーダの計測 (PCINT割り込み)
//   左: EL_A=PB0(PCINT0), EL_B=PD2(PCINT18)
//   右: ER_A=PD3(PCINT19), ER_B=PD4(PCINT20)
// いずれもPCINTで統一して読み取り、標準的な4遷移デコードテーブルで
// 増減方向を判定し、グローバルなカウンタに積算する。

#include <Arduino.h>

// 現在の積算カウント値 (符号がそのまま回転方向)
extern volatile int32_t encoderLeftCount;
extern volatile int32_t encoderRightCount;

void encoderInit();
