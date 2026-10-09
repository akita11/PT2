#include <Arduino.h>
#include "pins.h"
#include "resistor_matrix.h"
#include "motor_control.h"
#include "encoder.h"
#include "ws2812_control.h"
#include "motion_control.h"
#include "program_runner.h"
#include "sw.h"

void setup() {
  Serial.begin(9600);
  swInit();
  resistorMatrixInit();
  motorControlInit();
  encoderInit();
  motionControlInit();
  ledInit();

  Serial.println(F("PT2 firmware ready."));
}
/*
ギヤ比=1:298
Enc: 7pulse/rot
298*7 = 2086pulse/rot
*/

void loop() {
	if (swRead()) {
    while (swRead()) {
      delay(10);
    }
    delay(50); // 離した直後のチャタリングで実行中断と誤判定しないよう待つ
		encoderLeftCount = 0;
		encoderRightCount = 0;
		motionIsrMaxMicros(true);
	  moveByPulses(2086, 2086, 100);
  	motionWaitComplete();
	  Serial.print(encoderLeftCount); Serial.print(","); Serial.println(encoderRightCount);
	  motionPrintResult();
	  Serial.print(F("ISR max[us]=")); Serial.println(motionIsrMaxMicros());
	  //programRun();
	}
/*
  // for encoder count test
	if (swRead()){
    setMotorLeft(100); setMotorRight(100);
  }
  else{    
    setMotorLeft(0); setMotorRight(0);
  }
  Serial.print(encoderLeftCount); Serial.print(","); Serial.println(encoderRightCount);
	*/
}
