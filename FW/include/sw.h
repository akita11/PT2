#pragma once
// スタートスイッチ(PIN_SW)の読み取り。INPUT_PULLUPのためactive-low
// (押下時にLOW)であり、swRead()はこれを吸収して押下時にtrueを返す。

#include <Arduino.h>

void swInit();

bool swRead();
