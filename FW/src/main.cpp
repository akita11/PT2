#include <Arduino.h>
#include "pins.h"
#include "resistor_matrix.h"
#include "motor_control.h"
#include "encoder.h"
#include "ws2812_control.h"
#include "motion_control.h"
#include "sw.h"

void setup() {
  Serial.begin(38400);

  swInit();

  resistorMatrixInit();
  motorControlInit();
  encoderInit();
  motionControlInit();
  ledInit();

  Serial.println(F("PT2 firmware ready."));
}

void loop() {
  resistorMatrixScan();
  resistorMatrixPrint();
  resistorMatrixPrintResistance();
  delay(500);
  // LEDデモ: 全点灯を赤にして表示
/*
  ledSetAll(16, 0, 0);
  ledShow();
*/
  if (swRead()) {
    while(swRead()) {
      delay(10);
    }
    Serial.print(F("EncoderL=")); Serial.print(encoderLeftCount); Serial.print(F(" EncoderR=")); Serial.println(encoderRightCount);
    moveByPulses(1000, 1000, 100); // dL, dR, maxSpeed
    motionWaitComplete();
    delay(1000);
    Serial.print(F("EncoderL=")); Serial.print(encoderLeftCount); Serial.print(F(" EncoderR=")); Serial.println(encoderRightCount);
  }
}
