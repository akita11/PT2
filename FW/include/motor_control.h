#pragma once
// DCモータ PWM制御 (AD116ドライバ, AIN/BIN = PWM+方向の2線インターフェース)
//   speed > 0 : 正転 (AIN=PWM, BIN=LOW)
//   speed < 0 : 逆転 (AIN=LOW, BIN=PWM)
//   speed = 0 : コースト (AIN=BIN=LOW)
//   ブレーキ  : AIN=BIN=HIGH (setMotorXxx()で速度を与えるまで保持)

#include <Arduino.h>

void motorControlInit();

// speed範囲: -255 .. 255
void setMotorLeft(int16_t speed);
void setMotorRight(int16_t speed);

void brakeMotorLeft();
void brakeMotorRight();
