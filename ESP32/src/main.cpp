#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>
#include <cstdlib>

namespace {
constexpr char kBleDeviceName[] = "ESP32-S3-DEVKITC-1-N16R8V";
constexpr char kServiceUuid[] = "12345678-1234-1234-1234-1234567890ab";
constexpr char kControlCharUuid[] = "12345678-1234-1234-1234-1234567890ac";

// ESP32-S3-DevKitC-1 variants commonly expose onboard RGB LED on GPIO48.
constexpr uint8_t kRgbPin = 48;
constexpr uint8_t kPixelCount = 1;

Adafruit_NeoPixel gPixel(kPixelCount, kRgbPin, NEO_GRB + NEO_KHZ800);
NimBLEServer* gServer = nullptr;

void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  gPixel.setPixelColor(0, gPixel.Color(red, green, blue));
  gPixel.show();
}

uint8_t lerpColor(uint8_t from, uint8_t to, float t) {
  const float clamped = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
  return static_cast<uint8_t>(from + (to - from) * clamped);
}

void setLedFromOmega(float omega) {
  if (omega < -1.0f) omega = -1.0f;
  if (omega > 1.0f) omega = 1.0f;

  if (omega < 0.0f) {
    // -1.0 -> green, 0.0 -> blue
    const float t = omega + 1.0f;
    setLedColor(0, lerpColor(255, 0, t), lerpColor(0, 255, t));
  } else {
    // 0.0 -> blue, 1.0 -> red
    const float t = omega;
    setLedColor(lerpColor(0, 255, t), 0, lerpColor(255, 0, t));
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    // Blue when app is connected via BLE.
    setLedColor(0, 0, 255);
    Serial.printf("BLE connected, active links: %d\n", server->getConnectedCount());
  }

  void onDisconnect(NimBLEServer* server) override {
    // Back to red when disconnected and continue advertising.
    setLedColor(255, 0, 0);
    NimBLEDevice::startAdvertising();
    Serial.printf("BLE disconnected, active links: %d\n", server->getConnectedCount());
  }
};

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    const std::string payload = characteristic->getValue();
    Serial.print("BLE RX: ");
    Serial.println(payload.c_str());

    if (payload.rfind("OMEGA:", 0) == 0) {
      const float omega = std::atof(payload.c_str() + 6);
      setLedFromOmega(omega);
      return;
    }
  }
};

ServerCallbacks gServerCallbacks;
ControlCallbacks gControlCallbacks;
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);

  gPixel.begin();
  gPixel.setBrightness(40);
  setLedColor(255, 0, 0);

  NimBLEDevice::init(kBleDeviceName);
  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(&gServerCallbacks);

  NimBLEService* service = gServer->createService(kServiceUuid);
  NimBLECharacteristic* controlCharacteristic = service->createCharacteristic(
      kControlCharUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);

  controlCharacteristic->setCallbacks(&gControlCallbacks);
  controlCharacteristic->setValue("ready");
  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(kBleDeviceName);
  advertising->addServiceUUID(kServiceUuid);
  advertising->start();

  Serial.println("BLE server started");
  Serial.print("Advertising as: ");
  Serial.println(kBleDeviceName);
  Serial.println("LED red: waiting for app connection");
}

void loop() {
  delay(20);
}
