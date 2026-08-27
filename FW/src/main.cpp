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
/*
  resistorMatrixScan();
  resistorMatrixPrint();

*/
/*
  // LEDデモ: 全点灯を赤にして表示
  ledSetAll(16, 0, 0);
  ledShow();
*/
/*
  // motor & encoder test
  Serial.print(motionIsActive() ? F("Moving ") : F("Stopped "));
  Serial.print(F("EncoderL="));
  Serial.print(encoderLeftCount);
  Serial.print(F(" EncoderR="));
  Serial.println(encoderRightCount);
*/
  if (swRead()) {
    while(swRead()) {
      delay(10);
    }
//    Serial.println(F("Button pressed. Move 1000 pulses."));
    Serial.println(encoderRightCount);
    moveByPulses(1000, 5000, 100, 100); // dL, dR, maxSpeed, slope
    motionWaitComplete();
    delay(1000);
    Serial.println(encoderRightCount);
    // memo: 10%くらい多めに回ってしまう
  }
}
