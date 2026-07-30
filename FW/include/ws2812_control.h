#pragma once
// WS2812 x16 (LEDCネット, PC5) 制御。Adafruit_NeoPixelライブラリを使用。

#include <Arduino.h>

void ledInit();
void ledSetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void ledSetAll(uint8_t r, uint8_t g, uint8_t b);
void ledSetBrightness(uint8_t brightness); // 0-255
void ledClear();
void ledShow();
