#pragma once
// モータに連動する2相ロータリエンコーダの計測 (PCINT割り込み)
//   左: EL_A=PB0(PCINT0), EL_B=PD2(PCINT18)
//   右: ER_A=PD3(PCINT19), ER_B=PD4(PCINT20)
// いずれもPCINTで統一して読み取り、標準的な4遷移デコードテーブルで
// 増減方向を判定し、グローバルなカウンタに積算する。

#include <Arduino.h>

// 定義すると、A相の立ち上がりエッジのみでカウントする(分解能は1/4)。
// この場合、カウント値は回転方向によらず増加のみとなる。
//   左: EL_A=PD3 -> INT1(立ち上がり), 右: ER_A=PB0 -> ICP1(Timer1入力キャプチャ, 立ち上がり)
// コメントアウトすると従来の2相(4逓倍)デコードになる。
#define ENCODER_SINGLE_PHASE

// 現在の積算カウント値 (2相デコード時は符号がそのまま回転方向)
extern volatile int32_t encoderLeftCount;
extern volatile int32_t encoderRightCount;

void encoderInit();
