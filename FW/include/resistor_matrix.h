#pragma once
// 4x4抵抗マトリクス スキャン+5クラス判定
// (resistor_matrix_scan_classify.ino を PT2実基板の配線に合わせて移植)
//
// 実配線では ResCol0-3 が駆動(OUTPUT)、ResRow0-3 が検出(ADC入力)であり、
// 元スケッチの行/列の役割が入れ替わっている点に注意 (pins.h参照)。
// 分圧・判定の数式自体は対称な回路トポロジーのため元のまま流用できる。

#include <Arduino.h>

constexpr uint8_t RM_N_CLASSES = 5;
extern const float RM_R_CLASS[RM_N_CLASSES];
extern const char *const RM_R_CLASS_LABEL[RM_N_CLASSES];

// 結果格納用 (行=ResRow index, 列=ResCol index)
extern float rmResistance[4][4];   // 推定抵抗値 [Ω] (表示/参考用)
extern int8_t rmClass[4][4];       // 判定クラス番号(0-4)、判定不能なら-1

void resistorMatrixInit();
void resistorMatrixScan();
void resistorMatrixPrint();
