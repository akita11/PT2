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

volatile uint8_t stMotor = 0;

void loop() {
/*
  resistorMatrixScan();
  resistorMatrixPrint();

  Serial.print(F("EncoderL="));
  Serial.print(encoderLeftCount);
  Serial.print(F(" EncoderR="));
  Serial.println(encoderRightCount);
*/
/*
  // LEDデモ: 全点灯を赤にして表示
  ledSetAll(16, 0, 0);
  ledShow();
*/

  // motor & encoder test
  Serial.print(F("EncoderL="));
  Serial.print(encoderLeftCount);
  Serial.print(F(" EncoderR="));
  Serial.println(encoderRightCount);

  switch(stMotor) {
    case 0:
      setMotorLeft(100);
      setMotorRight(100);
      break;
    case 1:
      setMotorLeft(0);
      setMotorRight(0);
      break;
    case 2:
      setMotorLeft(-100);
      setMotorRight(-100);
      break;
    case 3:
      setMotorLeft(-200);
      setMotorRight(-200);
      break;
  }
  stMotor = (stMotor + 1) % 4;

  delay(1000);
}
