#pragma once
// エンコーダフィードバックによる、パルス数指定のPID位置制御。
// Timer2 (CTC, 2ms周期) の割り込みで、左右独立に目標パルス数との誤差を
// PID演算し、モータ速度に変換し続ける(非ブロッキング)。
// 行き過ぎた場合は誤差の符号が反転するため、自動的に逆転して戻る。
// ただしENCODER_SINGLE_PHASE(encoder.h)定義時はカウントが増加のみのため戻しの
// 制御は行わず、ブレーキ後の惰性で進む分(軸ごとのcoast)だけ目標手前でブレーキをかける。
// 目標の手前(decelDist)からは出力上限を下げて減速し、到達したらブレーキ(AIN=BIN=H)を
// かける。ブレーキは次の移動指令まで保持する。
// PIDゲイン(左右別)・減速・惰性補正の初期値などのチューニング値は motion_control.cpp 内の
// GAIN_L/GAIN_R を参照。

#include <Arduino.h>

// ENCODER_SINGLE_PHASE時、移動ごとに停止後の行き過ぎ量を実測し、惰性補正量(coast)を
// 自動更新する。学習値はRAM上のみで、リセットすると motion_control.cpp の初期値に戻る。
// 惰性量は速度(maxSpeed)や負荷で変わるため、同じ条件の移動を繰り返す場合に有効。
#define MOTION_COAST_LEARNING

void motionControlInit();

// encoderLeftCount/encoderRightCount の現在値からの増減パルス数(delta)を指定して
// 左右モータをPID制御で目標パルス数まで移動させる。呼び出すと即座にリターンし、
// 以後はTimer2割り込みがバックグラウンドで速度を更新する。
//   deltaLeft/deltaRight : 目標との差分パルス数 (符号が回転方向、0ならその軸は動かさない)
//   maxSpeed              : 出力速度の上限 (PWM 0-255)
//   tolerance             : 制御完了とみなす目標との差の許容範囲[パルス] (デフォルト10)
//                           ENCODER_SINGLE_PHASE時は使わない(惰性補正量coastで停止判定)
void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, int32_t tolerance = 10);

// 左右いずれかの軸が移動中、または停止指令後の惰性が収まっていない(カウントが
// 100ms変化しなくなるまで)なら true
bool motionIsActive();

// 移動完了(惰性の停止まで含む)までブロックして待つ(任意で使用)
void motionWaitComplete();

// 移動を即時中断してブレーキをかける(惰性の停止は待たず、学習も行わない)
void motionStop();

// チューニング確認用: 直前の移動の結果を左右それぞれSerialに出力する。
//   target: 目標カウント, stop: ブレーキ指令時のカウント, final: 惰性停止後のカウント,
//   over: final-target, coast: 次回の移動で使う惰性補正量
void motionPrintResult();

// チューニング確認用: Timer2割り込み(PID演算)の最大処理時間[us]を返す(8us単位)。
// reset=trueで最大値をクリアする。制御周期(2ms)を超えるとTCNT2が一周するため正しく測れない。
uint16_t motionIsrMaxMicros(bool reset = true);
