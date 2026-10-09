#include "motor_control.h"
#include "pins.h"

static void driveMotor(uint8_t pinA, uint8_t pinB, int16_t speed) {
  speed = constrain(speed, (int16_t)-255, (int16_t)255);
  if (speed > 0) {
    analogWrite(pinA, (uint8_t)speed);
    analogWrite(pinB, 0);
  } else if (speed < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, (uint8_t)(-speed));
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

// digitalWrite()はそのピンのPWM出力も停止する
static void brakeMotor(uint8_t pinA, uint8_t pinB) {
  digitalWrite(pinA, HIGH);
  digitalWrite(pinB, HIGH);
}

void motorControlInit() {
  pinMode(PIN_L_A, OUTPUT);
  pinMode(PIN_L_B, OUTPUT);
  pinMode(PIN_R_A, OUTPUT);
  pinMode(PIN_R_B, OUTPUT);
  setMotorLeft(0);
  setMotorRight(0);
}

void setMotorLeft(int16_t speed) {
  driveMotor(PIN_L_A, PIN_L_B, speed);
}

void setMotorRight(int16_t speed) {
  //driveMotor(PIN_R_A, PIN_R_B, speed);
  // reverse the direction of the right motor to match the left motor's direction
  driveMotor(PIN_R_B, PIN_R_A, speed);
}

void brakeMotorLeft() {
  brakeMotor(PIN_L_A, PIN_L_B);
}

void brakeMotorRight() {
  brakeMotor(PIN_R_A, PIN_R_B);
}
