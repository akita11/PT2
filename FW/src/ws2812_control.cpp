#include "ws2812_control.h"
#include "pins.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel strip(LED_COUNT, PIN_LEDC, NEO_GRB + NEO_KHZ800);

void ledInit() {
  strip.begin();
  strip.clear();
  strip.show();
}

void ledSetPixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= LED_COUNT) return;
  strip.setPixelColor(index, strip.Color(r, g, b));
}

void ledSetAll(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
}

void ledSetBrightness(uint8_t brightness) {
  strip.setBrightness(brightness);
}

void ledClear() {
  strip.clear();
}

void ledShow() {
  strip.show();
}
