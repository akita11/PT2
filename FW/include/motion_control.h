#pragma once
// エンコーダフィードバックによる、パルス数指定のPID位置制御。
// Timer2 (CTC, 1ms周期) の割り込みで、左右独立に目標パルス数との誤差を
// PID演算し、モータ速度に変換し続ける(非ブロッキング)。
// 行き過ぎた場合は誤差の符号が反転するため、自動的に逆転して戻る。
// PIDゲイン・許容誤差などのチューニング値は motion_control.cpp 内の定数を参照。

#include <Arduino.h>

void motionControlInit();

// encoderLeftCount/encoderRightCount の現在値からの増減パルス数(delta)を指定して
// 左右モータをPID制御で目標パルス数まで移動させる。呼び出すと即座にリターンし、
// 以後はTimer2割り込みがバックグラウンドで速度を更新する。
// 目標との差の絶対値がtolerance以下になった時点で、その軸は制御完了とする。
//   deltaLeft/deltaRight : 目標との差分パルス数 (符号が回転方向、0ならその軸は動かさない)
//   maxSpeed              : 出力速度の上限 (PWM 0-255)
//   tolerance             : 制御完了とみなす目標との差の許容範囲[パルス] (デフォルト10)
void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, int32_t tolerance = 10);

// 左右いずれかの軸が移動中(目標パルス数との差がtolerance以下になっていない)なら true
bool motionIsActive();

// 移動完了までブロックして待つ(任意で使用)
void motionWaitComplete();

// 移動を即時中断してモータを停止する
void motionStop();
