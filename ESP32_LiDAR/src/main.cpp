#include <Arduino.h>

namespace {
#if defined(LED_BUILTIN)
constexpr int kLedPin = LED_BUILTIN;
#else
constexpr int kLedPin = 2;
#endif

constexpr uint32_t kBlinkIntervalMs = 500;
uint32_t gLastBlinkMs = 0;
bool gLedOn = false;
}

void setup() {
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);
}

void loop() {
  const uint32_t now = millis();
  if (now - gLastBlinkMs >= kBlinkIntervalMs) {
    gLastBlinkMs = now;
    gLedOn = !gLedOn;
    digitalWrite(kLedPin, gLedOn ? HIGH : LOW);
  }
}
