#include "motion_control.h"
#include "encoder.h"
#include "motor_control.h"
#include <avr/interrupt.h>

// 起動直後・目標直前でのモータ最低速度。位置ベースの台形プロファイルは
// 走行パルス数=0の地点で速度=0になってしまい動き出せないため、下駄として加える。
//static constexpr uint8_t MOTION_MIN_SPEED = 40;
static constexpr uint8_t MOTION_MIN_SPEED = 60;

struct MotionAxis {
  volatile bool active;
  int32_t startCount;
  int32_t targetCount;
  int8_t dir; // +1 or -1
};

static volatile MotionAxis axisL = {false, 0, 0, 1};
static volatile MotionAxis axisR = {false, 0, 0, 1};

static volatile uint8_t gMaxSpeed = 0;
static volatile uint8_t gSlope = 0;

static void updateAxis(volatile MotionAxis &axis, volatile int32_t &count, void (*setSpeed)(int16_t)) {
  if (!axis.active) {
    return;
  }

  int32_t traveled = (count - axis.startCount) * axis.dir;
  int32_t remaining = (axis.targetCount - count) * axis.dir;
/*
  Serial.print("traveled=");
  Serial.print(traveled);
  Serial.print(" remaining=");
  Serial.println(remaining);
*/
  if (remaining <= 0) {
    setSpeed(0);
    axis.active = false;
    return;
  }

  int32_t vAccel = (int32_t)MOTION_MIN_SPEED + (int32_t)gSlope * traveled;
  int32_t vDecel = (int32_t)MOTION_MIN_SPEED + (int32_t)gSlope * remaining;
  int32_t v = (vAccel < vDecel) ? vAccel : vDecel;
  if (v > gMaxSpeed) {
    v = gMaxSpeed;
  }
//  Serial.printf("%d\n", (int16_t )(v * axis.dir));
  setSpeed((int16_t)(v * axis.dir));
}

ISR(TIMER2_COMPA_vect) {
//  updateAxis(axisL, encoderLeftCount, setMotorLeft);
  updateAxis(axisR, encoderRightCount, setMotorRight);
}

void motionControlInit() {
  axisL.active = false;
  axisR.active = false;

  noInterrupts();
  TCCR2A = (1 << WGM21);  // CTC (OCR2Aをtopとする)
  TCCR2B = (1 << CS22);   // prescaler 64
  OCR2A = 124;            // 8MHz/64/(124+1) = 1000Hz -> 1ms周期
  TCNT2 = 0;
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

void moveByPulses(int32_t deltaLeft, int32_t deltaRight, uint8_t maxSpeed, uint8_t slope) {
  noInterrupts();
  gMaxSpeed = maxSpeed;
  gSlope = slope;

  axisL.startCount = encoderLeftCount;
  axisL.targetCount = encoderLeftCount + deltaLeft;
  axisL.dir = (deltaLeft >= 0) ? 1 : -1;
  axisL.active = (deltaLeft != 0);

  axisR.startCount = encoderRightCount;
  axisR.targetCount = encoderRightCount + deltaRight;
  axisR.dir = (deltaRight >= 0) ? 1 : -1;
  axisR.active = (deltaRight != 0);
  interrupts();

  if (!axisL.active) {
    setMotorLeft(0);
  }
  if (!axisR.active) {
    setMotorRight(0);
  }
}

bool motionIsActive() {
//  return axisL.active || axisR.active;
  return axisR.active;
}

void motionWaitComplete() {
  while (motionIsActive()) {
    // 完了までポーリング待機
  }
}

void motionStop() {
  noInterrupts();
  axisL.active = false;
  axisR.active = false;
  interrupts();
  setMotorLeft(0);
  setMotorRight(0);
}
