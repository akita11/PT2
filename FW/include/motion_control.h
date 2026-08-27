#pragma once
// エンコーダフィードバックによる、パルス数指定・台形加減速のモータ位置制御。
// Timer2 (CTC, 1ms周期) の割り込みで、左右独立に走行/残りパルス数から
// 目標速度を再計算し続ける(非ブロッキング)。
//   speed = min(maxSpeed, MIN_SPEED + slope*走行パルス数, MIN_SPEED + slope*残りパルス数)
// という位置ベースのクランプにより、加速→巡航→減速の台形プロファイルを実現する。

#include <Arduino.h>

void motionControlInit();

// encoderLeftCount/encoderRightCount の現在値からの増減パルス数(delta)を指定して
// 左右モータを台形加減速で移動させる。呼び出すと即座にリターンし、以後はTimer2割り込みが
// バックグラウンドで速度を更新する。
//   deltaLeft/deltaRight : 目標との差分パルス数 (符号が回転方向、0ならその軸は動かさない)
//   maxSpeed             : 巡航速度 (PWM 0-255)
//   slope                : 加減速の傾き (走行/残りパルス数1あたりのPWM増加量、大きいほど急加減速)
void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, uint8_t slope);

// 左右いずれかの軸が移動中(目標パルス数に未到達)なら true
bool motionIsActive();

// 移動完了までブロックして待つ(任意で使用)
void motionWaitComplete();

// 移動を即時中断してモータを停止する
void motionStop();
