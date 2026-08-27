#include "sw.h"
#include "pins.h"

void swInit() {
  pinMode(PIN_SW, INPUT_PULLUP);
}

bool swRead() {
  return digitalRead(PIN_SW) == LOW;
}
