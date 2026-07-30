#pragma once
// PT2.kicad_sch (U3 = ATmega328P-A) のネット名にあわせたピン定義。
// Arduinoピン番号は標準の bare-328P マッピング
// (D0-D7=PD0-7, D8-D13=PB0-5, A0-A5=PC0-5) に基づく。

#include <Arduino.h>

// ---- 抵抗マトリクス (4x4) ----
// 実配線では ADC対応ピン(PC0-3) が行(ResRow)側、列(ResCol)側は ADC非対応
// (PB3/PB4/PB5, PD7) のため、元スケッチ(行=駆動/列=検出)と役割を入れ替える:
//   ResCol0-3 : デジタル出力で1本ずつHIGH駆動
//   ResRow0-3 : analogRead()で検出
// ResCol0-2 (D11-D13) はISPヘッダ(MOSI/MISO/SCK)と共用。初回ISP書込み時のみ
// 意味が変わる(問題はないが把握しておくこと)。
constexpr uint8_t PIN_RESCOL0 = 11; // PB3 / MOSI
constexpr uint8_t PIN_RESCOL1 = 12; // PB4 / MISO
constexpr uint8_t PIN_RESCOL2 = 13; // PB5 / SCK
constexpr uint8_t PIN_RESCOL3 = 7;  // PD7

constexpr uint8_t PIN_RESROW0 = A0; // PC0 / ADC0
constexpr uint8_t PIN_RESROW1 = A1; // PC1 / ADC1
constexpr uint8_t PIN_RESROW2 = A2; // PC2 / ADC2
constexpr uint8_t PIN_RESROW3 = A3; // PC3 / ADC3

// ---- モータ (AD116ドライバ、AIN/BIN = PWM/方向の2線インターフェース) ----
constexpr uint8_t PIN_L_A = 5;  // PD5 / OC0B - 左モータ AIN
constexpr uint8_t PIN_L_B = 6;  // PD6 / OC0A - 左モータ BIN
constexpr uint8_t PIN_R_A = 9;  // PB1 / OC1A - 右モータ AIN
constexpr uint8_t PIN_R_B = 10; // PB2 / OC1B - 右モータ BIN

// ---- ロータリエンコーダ (2相、PCINTで計測) ----
constexpr uint8_t PIN_EL_A = 8; // PB0 / PCINT0  - 左エンコーダA
constexpr uint8_t PIN_EL_B = 2; // PD2 / PCINT18 - 左エンコーダB
constexpr uint8_t PIN_ER_A = 3; // PD3 / PCINT19 - 右エンコーダA
constexpr uint8_t PIN_ER_B = 4; // PD4 / PCINT20 - 右エンコーダB

// ---- LED (WS2812 x16, 単線シリアルチェーン) ----
constexpr uint8_t PIN_LEDC = A5; // PC5
constexpr uint8_t LED_COUNT = 16;

// ---- その他 ----
constexpr uint8_t PIN_SW = A4; // PC4 - スタートスイッチ
