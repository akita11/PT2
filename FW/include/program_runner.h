#pragma once
// 並べたブロック(抵抗マトリクス)を読み取り、所定の動作を順に実行する。
// 仕様は motion_definition.md / LED_definition.md を参照。
//
//       col=0     1     2     3
// row=0     loop1 loop2 loop3 loop4   (サブルーチン)
// row=1     op1   op2   op3   op4
// row=2     op5   op6   op7   op8
// row=3     op9   op10  op11  op12
//
// op1から、置かれている最後のブロックまでを順に実行する。
// loopブロックはloop1-loop4のサブルーチンコール(loop行の最後のブロックまで実行して復帰)。
// 途中にブロックが置かれていない位置があれば、そのLEDをLED_BLINK_CYCLE_ERROR周期で
// 点滅させて一時停止し、ブロックが置かれたらLEDを点灯に変えてWAIT_AFTER_BLOCK_PLACEDだけ
// 待ってから実行を継続する。

#include <Arduino.h>

// ---- 機体パラメータ ----
#define WHEEL_DIAMETER 28.0          // タイヤ直径[mm]
#define WHEEL_TREAD 108.0            // 左右タイヤの間隔[mm]
#define ENCODER_PULSES_PER_REV 2086  // タイヤ1回転あたりのエンコーダパルス数 (ギヤ比298 x 7pulse)

// タイヤの移動距離[mm]をエンコーダのカウント値に換算する(四捨五入)
#define MM_TO_COUNT(mm) ((int32_t)((mm) * ENCODER_PULSES_PER_REV / (M_PI * WHEEL_DIAMETER) + 0.5))

// ---- 動作量 ----
//#define STEP_MOVE 150.0 // forward/backwardの移動距離[mm]
#define STEP_MOVE 100.0 // forward/backwardの移動距離[mm]
#define COUNT_STEP_MOVE MM_TO_COUNT(STEP_MOVE) // forward/backwardの移動量[パルス] (100mmで約2371)
// その場で90度回転する際、各タイヤが円周(π*WHEEL_TREAD)の1/4だけ逆向きに進む量[パルス]
// (WHEEL_TREAD=108mmで約2012)
#define STEP_ROTATE MM_TO_COUNT(M_PI * WHEEL_TREAD / 4.0)

#define LED_BLINK_CYCLE 500       // 実行中ブロックのLED点滅周期[ms]
#define LED_BLINK_CYCLE_ERROR 200 // ブロック未配置位置で待機中のLED点滅周期[ms]
#define WAIT_AMONG_OPERATION 500  // 各動作の間の待機時間[ms]
#define WAIT_AFTER_BLOCK_PLACED 500 // ブロック未配置位置にブロックが置かれてから動作を継続するまでの待機時間[ms]

// ブロック種別。値はrmClass(抵抗値のクラス番号)と一致させる。
enum BlockOp : int8_t {
  OP_NONE = -1,
  OP_RIGHT = 0,    // 2.2k
  OP_LEFT = 1,     // 4.7k
  OP_FORWARD = 2,  // 10k
  OP_BACKWARD = 3, // 22k
  OP_LOOP = 4,     // 47k
};

// 起動時のLED演出(50msごとに1個ずつのみ、ランダムな色で順に点灯し、最後に全消灯)。ledInit()の後に呼ぶ。
void programStartupLed();

// ブロックを読み取り、プログラム全体を実行し終えるまでブロックする。
// 実行中にSWが押された場合は即座に中断する。正常終了時は全LEDを青で3回点滅させる。
void programRun();
