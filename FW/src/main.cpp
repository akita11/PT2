#include <Arduino.h>
#include "pins.h"
#include "resistor_matrix.h"
#include "motor_control.h"
#include "encoder.h"
#include "ws2812_control.h"

void setup() {
  Serial.begin(38400);

  pinMode(PIN_SW, INPUT_PULLUP);

  resistorMatrixInit();
  motorControlInit();
  encoderInit();
  ledInit();

  Serial.println(F("PT2 firmware ready."));
}

void loop() {
  resistorMatrixScan();
  resistorMatrixPrint();

  Serial.print(F("EncoderL="));
  Serial.print(encoderLeftCount);
  Serial.print(F(" EncoderR="));
  Serial.println(encoderRightCount);

  // モータは安全のためデフォルトで停止。使用例:
  //   setMotorLeft(150);   // 左モータ正転
  //   setMotorRight(-150); // 右モータ逆転
  setMotorLeft(0);
  setMotorRight(0);

  // LEDデモ: 全点灯を赤にして表示
  ledSetAll(16, 0, 0);
  ledShow();

  delay(1000);
}
