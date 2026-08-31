#pragma once
// PT2.kicad_sch (U3 = ATmega328P-A) のネット名にあわせたピン定義。
// Arduinoピン番号は標準の bare-328P マッピング
// (D0-D7=PD0-7, D8-D13=PB0-5, A0-A5=PC0-5) に基づく。

#include <Arduino.h>

// ---- 抵抗マトリクス (16ch、CD74HC4067等の16:1アナログマルチプレクサ経由) ----
// 全16箇所のセル抵抗の駆動側は3.3Vに直結(GPIO不要)。
// センス側16本をマルチプレクサでSIG 1本に集約し、SIG-GND間に共通の
// プルダウン抵抗(R_PD, 10kΩ)を1本だけ実装する。
// マルチプレクサのINHはGND直結で常時イネーブルとする想定。
// ISPヘッダ(MOSI/MISO/SCK, D11-D13)とは衝突しないようA1-A3に割当。
constexpr uint8_t PIN_RM_MUX_S0 = A2;  // PC2        - マルチプレクサ アドレスbit0
constexpr uint8_t PIN_RM_MUX_S1 = A1;  // PC1        - マルチプレクサ アドレスbit1
constexpr uint8_t PIN_RM_MUX_S2 = A3;  // PC3        - マルチプレクサ アドレスbit2
constexpr uint8_t PIN_RM_MUX_S3 = A4;  // PC4        - マルチプレクサ アドレスbit3
constexpr uint8_t PIN_RM_MUX_SIG = A0; // PC0 / ADC0 - マルチプレクサ共通出力(ADC入力)

// ---- モータ (AD116ドライバ、AIN/BIN = PWM/方向の2線インターフェース) ----
constexpr uint8_t PIN_R_A = 5;  // PD5 / OC0B - 左モータ AIN
constexpr uint8_t PIN_R_B = 6;  // PD6 / OC0A - 左モータ BIN
constexpr uint8_t PIN_L_A = 9;  // PB1 / OC1A - 右モータ AIN
constexpr uint8_t PIN_L_B = 10; // PB2 / OC1B - 右モータ BIN

// ---- ロータリエンコーダ (2相、PCINTで計測) ----
constexpr uint8_t PIN_ER_A = 8; // PB0 / PCINT0  - 左エンコーダA
constexpr uint8_t PIN_ER_B = 2; // PD2 / PCINT18 - 左エンコーダB
constexpr uint8_t PIN_EL_A = 3; // PD3 / PCINT19 - 右エンコーダA
constexpr uint8_t PIN_EL_B = 4; // PD4 / PCINT20 - 右エンコーダB

// ---- LED (WS2812 x16, 単線シリアルチェーン) ----
constexpr uint8_t PIN_LEDC = A5; // PC5
constexpr uint8_t LED_COUNT = 16;

// ---- その他 ----
constexpr uint8_t PIN_SW = 7;  // PD7 - スタートスイッチ
