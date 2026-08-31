#pragma once
// 16箇所のセル抵抗を16:1アナログマルチプレクサ(CD74HC4067等)経由でスキャン+5クラス判定。
//
// 全セル抵抗の駆動側は3.3Vに直結、センス側16本をマルチプレクサでSIG 1本に集約し、
// SIG-GND間の共通プルダウン抵抗(R_PD)との分圧で読み取る(pins.h参照)。
// 各チャネルは他チャネルと電気的に共有ノードを持たないため、複数セルを同時に
// 実装してもクロストーク(ゴースト)が発生しない。
// チャネル番号chと表示上の(row,col)の対応は ch = row*4+col (resistor_matrix.cpp参照)。

#include <Arduino.h>

constexpr uint8_t RM_N_CLASSES = 5;
extern const float RM_R_CLASS[RM_N_CLASSES];
extern const char *const RM_R_CLASS_LABEL[RM_N_CLASSES];

// 結果格納用 (ch = row*4+col の対応で格納)
extern float rmResistance[4][4];   // 推定抵抗値 [Ω] (表示/参考用)
extern int8_t rmClass[4][4];       // 判定クラス番号(0-4)、判定不能なら-1

void resistorMatrixInit();
void resistorMatrixScan();
void resistorMatrixPrint();
void resistorMatrixPrintResistance(); // rmResistance[][]の推定抵抗値[Ω]をデバッグ表示
