#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <NimBLEDevice.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
constexpr char kBleDeviceName[] = "ESP32-S3-DEVKITC-1-N16R8V";
constexpr char kServiceUuid[] = "12345678-1234-1234-1234-1234567890ab";
constexpr char kControlCharUuid[] = "12345678-1234-1234-1234-1234567890ac";

// Ustaw tutaj swoje GPIO zgodnie z podlaczeniem.
constexpr int kFrontLeftPwmPin = 16;   // Lewy przod: PWM
constexpr int kFrontLeftIn1Pin = 7;    // Lewy przod: IN1
constexpr int kFrontLeftIn2Pin = 15;   // Lewy przod: IN2

constexpr int kFrontRightPwmPin = 9;   // Prawy przod: PWM
constexpr int kFrontRightIn1Pin = 11;  // Prawy przod: IN1
constexpr int kFrontRightIn2Pin = 12;  // Prawy przod: IN2

constexpr int kRearLeftPwmPin = 4;   // Lewy tyl: PWM
constexpr int kRearLeftIn1Pin = 6;   // Lewy tyl: IN1
constexpr int kRearLeftIn2Pin = 5;   // Lewy tyl: IN2

constexpr int kRearRightPwmPin = 17;   // Prawy tyl: PWM
constexpr int kRearRightIn1Pin = 8;    // Prawy tyl: IN1
constexpr int kRearRightIn2Pin = 18;   // Prawy tyl: IN2

constexpr int kDriver1StandbyPin = 13;  // TB6612 nr 1: STBY
constexpr int kDriver2StandbyPin = 13;  // TB6612 nr 2: STBY

constexpr uint8_t kFrontLeftPwmChannel = 0;
constexpr uint8_t kFrontRightPwmChannel = 1;
constexpr uint8_t kRearLeftPwmChannel = 2;
constexpr uint8_t kRearRightPwmChannel = 3;

constexpr uint32_t kPwmFrequency = 20000;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr int kPwmMax = (1 << kPwmResolutionBits) - 1;
constexpr float kDeadband = 0.08f;
constexpr uint32_t kCommandTimeoutMs = 350;
constexpr uint8_t kRgbPin = 48;
constexpr uint8_t kRgbPixelCount = 1;
constexpr uint32_t kHeartbeatIntervalMs = 500;
constexpr uint32_t kTestPhaseDurationMs = 2000;
constexpr float kTestMoveSpeed = 0.8f;
constexpr uint32_t kEncoderReportIntervalMs = 250;
constexpr int32_t kEncoderCountsPerRevolution = 44;

// Ustaw piny enkoderow. -1 oznacza "nieuzywany".
constexpr int kFrontLeftEncAPin = 1;
constexpr int kFrontLeftEncBPin = 2;
constexpr int kFrontRightEncAPin = 47;
constexpr int kFrontRightEncBPin = 48;
constexpr int kRearLeftEncAPin = 42;  //good
constexpr int kRearLeftEncBPin = 41;  //good
constexpr int kRearRightEncAPin = 39; //good
constexpr int kRearRightEncBPin = 40; //good

constexpr bool kRgbPinConflictsWithEncoder =
  (kRgbPin == kFrontLeftEncAPin) || (kRgbPin == kFrontLeftEncBPin) ||
  (kRgbPin == kFrontRightEncAPin) || (kRgbPin == kFrontRightEncBPin) ||
  (kRgbPin == kRearLeftEncAPin) || (kRgbPin == kRearLeftEncBPin) ||
  (kRgbPin == kRearRightEncAPin) || (kRgbPin == kRearRightEncBPin);
constexpr bool kUseStatusLed = !kRgbPinConflictsWithEncoder;

struct MotorPins {
  const char* name;
  int pwmPin;
  int in1Pin;
  int in2Pin;
  uint8_t pwmChannel;
  bool invertDirection;
};

struct DriveCommand {
  float x = 0.0f;
  float y = 0.0f;
  float omega = 0.0f;
};

struct EncoderConfig {
  const char* name;
  int pinA;
  int pinB;
  bool invertDirection;
};

struct EncoderState {
  volatile int32_t count = 0;
  int32_t lastCount = 0;
  bool enabled = false;
};

constexpr MotorPins kFrontLeftMotor{"Przednie lewe", kFrontLeftPwmPin,
                                    kFrontLeftIn1Pin, kFrontLeftIn2Pin,
                                    kFrontLeftPwmChannel, true};
constexpr MotorPins kFrontRightMotor{"Przednie prawe", kFrontRightPwmPin,
                                     kFrontRightIn1Pin, kFrontRightIn2Pin,
                                     kFrontRightPwmChannel, false};
constexpr MotorPins kRearLeftMotor{"Tylne lewe", kRearLeftPwmPin,
                                   kRearLeftIn1Pin, kRearLeftIn2Pin,
                                   kRearLeftPwmChannel, true};
constexpr MotorPins kRearRightMotor{"Tylne prawe", kRearRightPwmPin,
                                    kRearRightIn1Pin, kRearRightIn2Pin,
                                    kRearRightPwmChannel, false};

constexpr MotorPins kMotors[] = {
    kFrontLeftMotor,
    kFrontRightMotor,
    kRearLeftMotor,
    kRearRightMotor,
};

constexpr EncoderConfig kFrontLeftEncoder{"ENC FL", kFrontLeftEncAPin,
                      kFrontLeftEncBPin, false};
constexpr EncoderConfig kFrontRightEncoder{"ENC FR", kFrontRightEncAPin,
                       kFrontRightEncBPin, false};
constexpr EncoderConfig kRearLeftEncoder{"ENC RL", kRearLeftEncAPin,
                     kRearLeftEncBPin, false};
constexpr EncoderConfig kRearRightEncoder{"ENC RR", kRearRightEncAPin,
                      kRearRightEncBPin, false};

constexpr EncoderConfig kEncoders[] = {
  kFrontLeftEncoder,
  kFrontRightEncoder,
  kRearLeftEncoder,
  kRearRightEncoder,
};

NimBLEServer* gServer = nullptr;
DriveCommand gDriveCommand;
uint32_t gLastCommandMs = 0;
Adafruit_NeoPixel gRgb(kRgbPixelCount, kRgbPin, NEO_GRB + NEO_KHZ800);
bool gHeartbeatOn = false;
uint32_t gLastHeartbeatToggleMs = 0;
uint32_t gSequenceStartMs = 0;
bool gSequenceFinished = false;
EncoderState gEncoderStates[4];
uint32_t gLastEncoderReportMs = 0;

void setStatusLed(bool on) {
  if (!kUseStatusLed) {
    return;
  }

  if (on) {
    gRgb.setPixelColor(0, gRgb.Color(24, 0, 0));
  } else {
    gRgb.setPixelColor(0, gRgb.Color(0, 0, 0));
  }
  gRgb.show();
}

void updateHeartbeat() {
  if (!kUseStatusLed) {
    return;
  }

  const uint32_t now = millis();
  if (now - gLastHeartbeatToggleMs < kHeartbeatIntervalMs) {
    return;
  }

  gLastHeartbeatToggleMs = now;
  gHeartbeatOn = !gHeartbeatOn;
  setStatusLed(gHeartbeatOn);
}

float clampUnit(float value) {
  if (value < -1.0f) return -1.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float applyDeadband(float value) {
  const float clamped = clampUnit(value);
  return std::fabs(clamped) < kDeadband ? 0.0f : clamped;
}

void IRAM_ATTR handleEncoderEdge(size_t encoderIndex) {
  if (encoderIndex >= 4 || !gEncoderStates[encoderIndex].enabled) {
    return;
  }

  const EncoderConfig& encoder = kEncoders[encoderIndex];
  const bool a = digitalRead(encoder.pinA);
  const bool b = digitalRead(encoder.pinB);

  int32_t delta = (a == b) ? 1 : -1;
  if (encoder.invertDirection) {
    delta = -delta;
  }

  gEncoderStates[encoderIndex].count += delta;
}

void IRAM_ATTR encoderIsr0() { handleEncoderEdge(0); }
void IRAM_ATTR encoderIsr1() { handleEncoderEdge(1); }
void IRAM_ATTR encoderIsr2() { handleEncoderEdge(2); }
void IRAM_ATTR encoderIsr3() { handleEncoderEdge(3); }

void setupEncoders() {
  void (*isrHandlers[4])() = {encoderIsr0, encoderIsr1, encoderIsr2, encoderIsr3};

  for (size_t i = 0; i < 4; ++i) {
    const EncoderConfig& encoder = kEncoders[i];
    EncoderState& state = gEncoderStates[i];

    if (encoder.pinA < 0 || encoder.pinB < 0) {
      state.enabled = false;
      continue;
    }

    pinMode(encoder.pinA, INPUT_PULLUP);
    pinMode(encoder.pinB, INPUT_PULLUP);
    state.count = 0;
    state.lastCount = 0;
    state.enabled = true;

    attachInterrupt(digitalPinToInterrupt(encoder.pinA), isrHandlers[i], CHANGE);
    Serial.printf("%s configured on A=%d B=%d\n", encoder.name, encoder.pinA,
                  encoder.pinB);
  }
}

void reportEncoders() {
  const uint32_t now = millis();
  if (now - gLastEncoderReportMs < kEncoderReportIntervalMs) {
    return;
  }

  gLastEncoderReportMs = now;

  for (size_t i = 0; i < 4; ++i) {
    const EncoderConfig& encoder = kEncoders[i];
    EncoderState& state = gEncoderStates[i];

    if (!state.enabled) {
      continue;
    }

    noInterrupts();
    const int32_t currentCount = state.count;
    interrupts();

    const int32_t delta = currentCount - state.lastCount;
    state.lastCount = currentCount;

    const int pinAState = digitalRead(encoder.pinA);
    const int pinBState = digitalRead(encoder.pinB);
    const float rpm =
        (static_cast<float>(delta) * 60000.0f) /
        (static_cast<float>(kEncoderCountsPerRevolution) *
         static_cast<float>(kEncoderReportIntervalMs));

    if (delta != 0) {
      const char* direction = delta > 0 ? "PLUS" : "MINUS";
      Serial.printf("%s A=%d B=%d delta=%ld dir=%s count=%ld rpm=%.2f\n",
                    encoder.name, pinAState, pinBState, static_cast<long>(delta),
                    direction, static_cast<long>(currentCount), rpm);
    }
  }
}

void setDriverEnabled(bool enabled) {
  digitalWrite(kDriver1StandbyPin, enabled ? HIGH : LOW);
  digitalWrite(kDriver2StandbyPin, enabled ? HIGH : LOW);
}

void stopMotor(const MotorPins& motor) {
  digitalWrite(motor.in1Pin, LOW);
  digitalWrite(motor.in2Pin, LOW);
  ledcWrite(motor.pwmChannel, 0);
}

void stopAllMotors() {
  for (const MotorPins& motor : kMotors) {
    stopMotor(motor);
  }
}

void setMotorOutput(const MotorPins& motor, float command) {
  const float value = applyDeadband(command);
  if (value == 0.0f) {
    stopMotor(motor);
    return;
  }

  const bool desiredForward = value > 0.0f;
  const bool effectiveForward = motor.invertDirection ? !desiredForward : desiredForward;
  const int pwm = static_cast<int>(std::round(std::fabs(value) * kPwmMax));

  digitalWrite(motor.in1Pin, effectiveForward ? HIGH : LOW);
  digitalWrite(motor.in2Pin, effectiveForward ? LOW : HIGH);
  ledcWrite(motor.pwmChannel, pwm);
}

void applyDriveCommand(const DriveCommand& command) {
  float frontLeft = command.y + command.x + command.omega;
  float frontRight = command.y - command.x - command.omega;
  float rearLeft = command.y - command.x + command.omega;
  float rearRight = command.y + command.x - command.omega;

  const float maxMagnitude = std::max(
      1.0f,
      std::max(
          std::max(std::fabs(frontLeft), std::fabs(frontRight)),
          std::max(std::fabs(rearLeft), std::fabs(rearRight))));

  frontLeft /= maxMagnitude;
  frontRight /= maxMagnitude;
  rearLeft /= maxMagnitude;
  rearRight /= maxMagnitude;

  setMotorOutput(kFrontLeftMotor, frontLeft);
  setMotorOutput(kFrontRightMotor, frontRight);
  setMotorOutput(kRearLeftMotor, rearLeft);
  setMotorOutput(kRearRightMotor, rearRight);
}

bool parseDriveCommand(const std::string& payload, DriveCommand& command) {
  float x = 0.0f;
  float y = 0.0f;
  float omega = 0.0f;

  if (payload.rfind("DRIVE:", 0) == 0) {
    if (std::sscanf(payload.c_str() + 6, "%f,%f,%f", &x, &y, &omega) != 3) {
      return false;
    }
  } else if (payload.rfind("OMEGA:", 0) == 0) {
    omega = std::atof(payload.c_str() + 6);
  } else {
    return false;
  }

  command.x = clampUnit(x);
  command.y = clampUnit(y);
  command.omega = clampUnit(omega);
  return true;
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    Serial.printf("BLE connected, active links: %d\n", server->getConnectedCount());
  }

  void onDisconnect(NimBLEServer* server) override {
    stopAllMotors();
    gDriveCommand = DriveCommand{};
    NimBLEDevice::startAdvertising();
    Serial.printf("BLE disconnected, active links: %d\n", server->getConnectedCount());
  }
};

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    const std::string payload = characteristic->getValue();
    DriveCommand nextCommand;

    if (!parseDriveCommand(payload, nextCommand)) {
      Serial.printf("BLE RX bad payload: %s\n", payload.c_str());
      return;
    }

    gDriveCommand = nextCommand;
    gLastCommandMs = millis();
    applyDriveCommand(gDriveCommand);
  }
};

ServerCallbacks gServerCallbacks;
ControlCallbacks gControlCallbacks;
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);

  if (kUseStatusLed) {
    gRgb.begin();
    gRgb.setBrightness(30);
    setStatusLed(false);
  } else {
    Serial.printf(
        "Status LED disabled: RGB pin %u conflicts with encoder pin mapping\n",
        static_cast<unsigned>(kRgbPin));
  }
  gLastHeartbeatToggleMs = millis();

  pinMode(kDriver1StandbyPin, OUTPUT);
  pinMode(kDriver2StandbyPin, OUTPUT);
  setDriverEnabled(true);

  for (const MotorPins& motor : kMotors) {
    pinMode(motor.in1Pin, OUTPUT);
    pinMode(motor.in2Pin, OUTPUT);
    ledcSetup(motor.pwmChannel, kPwmFrequency, kPwmResolutionBits);
    ledcAttachPin(motor.pwmPin, motor.pwmChannel);
  }
  stopAllMotors();
  setupEncoders();

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

  gLastCommandMs = millis();
  gSequenceStartMs = millis();
  gSequenceFinished = false;
  gLastEncoderReportMs = millis();

  Serial.println("BLE DRIVE controller started");
  Serial.printf("Advertising as: %s\n", kBleDeviceName);
  Serial.println("Payload format: DRIVE:x,y,omega");
}

void loop() {
  updateHeartbeat();
  reportEncoders();

  const uint32_t elapsed = millis() - gSequenceStartMs;
  if (elapsed < kTestPhaseDurationMs) {
    DriveCommand forwardCommand;
    forwardCommand.x = 0.0f;
    forwardCommand.y = kTestMoveSpeed;
    forwardCommand.omega = 0.0f;
    applyDriveCommand(forwardCommand);
    delay(20);
    return;
  }

  if (elapsed < (2 * kTestPhaseDurationMs)) {
    DriveCommand backwardCommand;
    backwardCommand.x = 0.0f;
    backwardCommand.y = -kTestMoveSpeed;
    backwardCommand.omega = 0.0f;
    applyDriveCommand(backwardCommand);
    delay(20);
    return;
  }

  if (elapsed < (3 * kTestPhaseDurationMs)) {
    DriveCommand rightCommand;
    rightCommand.x = kTestMoveSpeed;
    rightCommand.y = 0.0f;
    rightCommand.omega = 0.0f;
    applyDriveCommand(rightCommand);
    delay(20);
    return;
  }

  if (elapsed < (4 * kTestPhaseDurationMs)) {
    DriveCommand leftCommand;
    leftCommand.x = -kTestMoveSpeed;
    leftCommand.y = 0.0f;
    leftCommand.omega = 0.0f;
    applyDriveCommand(leftCommand);
    delay(20);
    return;
  }

  if (!gSequenceFinished) {
    stopAllMotors();
    gSequenceFinished = true;
  }

  if (millis() - gLastCommandMs > kCommandTimeoutMs) {
    gDriveCommand = DriveCommand{};
    stopAllMotors();
  }

  delay(20);
}
