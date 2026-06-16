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
constexpr char kTelemetryCharUuid[] = "12345678-1234-1234-1234-1234567890ad";

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
constexpr bool kUseActiveBraking = true;
constexpr uint8_t kRgbPin = 48;
constexpr uint8_t kRgbPixelCount = 1;
constexpr uint32_t kHeartbeatIntervalMs = 500;
constexpr uint32_t kTestPhaseDurationMs = 2000;
constexpr float kTestMoveSpeed = 0.8f;
constexpr bool kRunStartupDriveSequence = false;
constexpr bool kEncoderTestOnly = false;
constexpr float kEncoderTestMotorCommand = 0.35f;
constexpr uint32_t kEncoderTestStepDurationMs = 2500;
constexpr uint32_t kControlIntervalMs = 20;
constexpr uint32_t kEncoderReportIntervalMs = 250;
constexpr uint32_t kTelemetryIntervalMs = 100;
constexpr bool kSerialPlotterTelemetry = true;
constexpr bool kSerialOnlyReceivedValues = true;
constexpr bool kUseEncoders = false;
constexpr int32_t kEncoderCountsPerRevolution = 44;
constexpr float kMaxWheelRpm = 220.0f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kWheelRadiusMeters = 0.050f;       // promien kola [m] (50 mm)
constexpr float kWheelbaseLengthMeters = 0.260f;   // odleglosc os przod-tyl [m] (260 mm)
constexpr float kTrackWidthMeters = 0.486f;        // odleglosc kol lewo-prawo [m] (486 mm)
constexpr float kWheelbaseRadiusMeters =
  (kWheelbaseLengthMeters * 0.5f) + (kTrackWidthMeters * 0.5f);
constexpr float kMaxWheelAngularRadPerSec = (kMaxWheelRpm * 2.0f * kPi) / 60.0f;
constexpr float kSafeWheelbaseRadiusMeters =
  (kWheelbaseRadiusMeters > 0.001f) ? kWheelbaseRadiusMeters : 0.001f;
constexpr float kMaxRobotVxMps =
  kMaxWheelAngularRadPerSec * kWheelRadiusMeters;  // +x = prawo [m/s]
constexpr float kMaxRobotVyMps =
  kMaxWheelAngularRadPerSec * kWheelRadiusMeters;  // +y = przod [m/s]
constexpr float kMaxRobotOmegaRadPerSec =
  (kMaxWheelAngularRadPerSec * kWheelRadiusMeters) /
  kSafeWheelbaseRadiusMeters;  // +omega = lewo (CCW) [rad/s]
constexpr float kSpeedKp = 0.00275f;
constexpr float kSpeedKi = 0.0100f;
constexpr float kFeedforwardGain = 0.75f;
constexpr float kIntegratorLimit = 0.60f;

// Ustaw piny enkoderow. -1 oznacza "nieuzywany".
constexpr int kFrontLeftEncAPin = 1;
constexpr int kFrontLeftEncBPin = 2;
constexpr int kFrontRightEncAPin = 47;
constexpr int kFrontRightEncBPin = 48;
constexpr int kRearLeftEncAPin = 42;
constexpr int kRearLeftEncBPin = 41;
constexpr int kRearRightEncAPin = 39;
constexpr int kRearRightEncBPin = 40;

constexpr bool kRgbPinConflictsWithEncoder =
  (kRgbPin == kFrontLeftEncAPin) || (kRgbPin == kFrontLeftEncBPin) ||
  (kRgbPin == kFrontRightEncAPin) || (kRgbPin == kFrontRightEncBPin) ||
  (kRgbPin == kRearLeftEncAPin) || (kRgbPin == kRearLeftEncBPin) ||
  (kRgbPin == kRearRightEncAPin) || (kRgbPin == kRearRightEncBPin);
constexpr bool kUseStatusLed = false;

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
  int32_t reportLastCount = 0;
  int32_t controlLastCount = 0;
  bool enabled = false;
};

struct WheelControlState {
  float targetNorm = 0.0f;
  float targetRpm = 0.0f;
  float measuredRpm = 0.0f;
  float integrator = 0.0f;
  float output = 0.0f;
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
NimBLECharacteristic* gControlCharacteristic = nullptr;
NimBLECharacteristic* gTelemetryCharacteristic = nullptr;
DriveCommand gDriveCommand;
uint32_t gLastCommandMs = 0;
Adafruit_NeoPixel gRgb(kRgbPixelCount, kRgbPin, NEO_GRB + NEO_KHZ800);
bool gHeartbeatOn = false;
uint32_t gLastHeartbeatToggleMs = 0;
uint32_t gSequenceStartMs = 0;
bool gSequenceFinished = false;
EncoderState gEncoderStates[4];
WheelControlState gWheelControl[4];
uint32_t gLastEncoderReportMs = 0;
uint32_t gLastControlMs = 0;
uint32_t gLastTelemetryMs = 0;
uint32_t gEncoderTestStartMs = 0;
int gLastEncoderTestPhase = -1;

constexpr const char* kWheelNames[4] = {
  "FRONT_LEFT",
  "FRONT_RIGHT",
  "REAR_LEFT",
  "REAR_RIGHT",
};

constexpr const char* kEncoderNames[4] = {
  "ENC FL",
  "ENC FR",
  "ENC RL",
  "ENC RR",
};

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
  if (!kUseEncoders) {
    for (EncoderState& state : gEncoderStates) {
      state.enabled = false;
      state.count = 0;
      state.reportLastCount = 0;
      state.controlLastCount = 0;
    }
    if (!kSerialOnlyReceivedValues) {
      Serial.println("Encoders disabled: open-loop mode");
    }
    return;
  }

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
    state.reportLastCount = 0;
    state.controlLastCount = 0;
    state.enabled = true;

    attachInterrupt(digitalPinToInterrupt(encoder.pinA), isrHandlers[i], CHANGE);
    if (!kSerialOnlyReceivedValues) {
      Serial.printf("%s configured on A=%d B=%d\n", encoder.name, encoder.pinA,
                    encoder.pinB);
    }
  }
}

void reportEncoders() {
  if (kSerialOnlyReceivedValues) {
    return;
  }

  const uint32_t now = millis();
  if (now - gLastEncoderReportMs < kEncoderReportIntervalMs) {
    return;
  }

  gLastEncoderReportMs = now;

  for (size_t i = 0; i < 4; ++i) {
    const EncoderConfig& encoder = kEncoders[i];
    EncoderState& state = gEncoderStates[i];

    if (!state.enabled) {
      Serial.printf("%s DISABLED\n", encoder.name);
      continue;
    }

    noInterrupts();
    const int32_t currentCount = state.count;
    interrupts();

    const int32_t delta = currentCount - state.reportLastCount;
    state.reportLastCount = currentCount;

    const int pinAState = digitalRead(encoder.pinA);
    const int pinBState = digitalRead(encoder.pinB);
    const float rpm =
        (static_cast<float>(delta) * 60000.0f) /
        (static_cast<float>(kEncoderCountsPerRevolution) *
         static_cast<float>(kEncoderReportIntervalMs));

    const char* direction = delta > 0 ? "PLUS" : (delta < 0 ? "MINUS" : "ZERO");
    Serial.printf("%s A=%d B=%d delta=%ld dir=%s count=%ld rpm_report=%.2f\n",
                  encoder.name, pinAState, pinBState, static_cast<long>(delta),
                  direction, static_cast<long>(currentCount), rpm);
  }
}

void sendBleTelemetry() {
  if (gTelemetryCharacteristic == nullptr) {
    return;
  }

  const uint32_t now = millis();
  if (now - gLastTelemetryMs < kTelemetryIntervalMs) {
    return;
  }
  gLastTelemetryMs = now;

  char payload[192];
  std::snprintf(
      payload, sizeof(payload),
      "T,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
      gWheelControl[0].targetRpm, gWheelControl[0].measuredRpm,
      gWheelControl[1].targetRpm, gWheelControl[1].measuredRpm,
      gWheelControl[2].targetRpm, gWheelControl[2].measuredRpm,
      gWheelControl[3].targetRpm, gWheelControl[3].measuredRpm);

  gTelemetryCharacteristic->setValue(payload);
  gTelemetryCharacteristic->notify();

  if (kSerialPlotterTelemetry && !kSerialOnlyReceivedValues) {
    Serial.printf(
        "fl_set:%.2f,fl_meas:%.2f,fr_set:%.2f,fr_meas:%.2f,rl_set:%.2f,rl_meas:%.2f,rr_set:%.2f,rr_meas:%.2f\n",
        gWheelControl[0].targetRpm, gWheelControl[0].measuredRpm,
        gWheelControl[1].targetRpm, gWheelControl[1].measuredRpm,
        gWheelControl[2].targetRpm, gWheelControl[2].measuredRpm,
        gWheelControl[3].targetRpm, gWheelControl[3].measuredRpm);
  }
}

void setDriverEnabled(bool enabled) {
  digitalWrite(kDriver1StandbyPin, enabled ? HIGH : LOW);
  digitalWrite(kDriver2StandbyPin, enabled ? HIGH : LOW);
}

void stopMotor(const MotorPins& motor) {
  if (kUseActiveBraking) {
    digitalWrite(motor.in1Pin, HIGH);
    digitalWrite(motor.in2Pin, HIGH);
    ledcWrite(motor.pwmChannel, kPwmMax);
  } else {
    digitalWrite(motor.in1Pin, LOW);
    digitalWrite(motor.in2Pin, LOW);
    ledcWrite(motor.pwmChannel, 0);
  }
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
  const float xNorm = clampUnit(command.x);      // +x = prawo
  const float yNorm = clampUnit(command.y);      // +y = przod
  const float omegaNorm = clampUnit(command.omega);  // +omega = lewo (CCW)

  const float vx = xNorm * kMaxRobotVxMps;
  const float vy = yNorm * kMaxRobotVyMps;
  const float omega = omegaNorm * kMaxRobotOmegaRadPerSec;

  const float safeWheelRadius = std::max(kWheelRadiusMeters, 0.001f);
  const float invWheelRadius = 1.0f / safeWheelRadius;

  // Mecanum inverse kinematics for frame: +x right, +y forward, +omega CCW.
  float wheelAngularRadPerSec[4] = {
      (vy + vx + (kWheelbaseRadiusMeters * omega)) * invWheelRadius,  // FL
      (vy - vx - (kWheelbaseRadiusMeters * omega)) * invWheelRadius,  // FR
      (vy - vx + (kWheelbaseRadiusMeters * omega)) * invWheelRadius,  // RL
      (vy + vx - (kWheelbaseRadiusMeters * omega)) * invWheelRadius,  // RR
  };

  float maxWheelAbs = 0.0f;
  for (float wheelSpeed : wheelAngularRadPerSec) {
    maxWheelAbs = std::max(maxWheelAbs, std::fabs(wheelSpeed));
  }

  const float safeMaxWheelAngular = std::max(kMaxWheelAngularRadPerSec, 0.001f);
  const float scale = maxWheelAbs > safeMaxWheelAngular
                          ? (safeMaxWheelAngular / maxWheelAbs)
                          : 1.0f;

  const float wheelRpmFactor = 60.0f / (2.0f * kPi);
  for (size_t i = 0; i < 4; ++i) {
    const float wheelRadPerSec = wheelAngularRadPerSec[i] * scale;
    gWheelControl[i].targetRpm = wheelRadPerSec * wheelRpmFactor;
    gWheelControl[i].targetNorm = clampUnit(gWheelControl[i].targetRpm / kMaxWheelRpm);
  }
}

void updateClosedLoopControl() {
  const uint32_t now = millis();
  const uint32_t dtMs = now - gLastControlMs;
  if (dtMs < kControlIntervalMs) {
    return;
  }
  gLastControlMs = now;

  const float dtSeconds = static_cast<float>(dtMs) / 1000.0f;

  for (size_t i = 0; i < 4; ++i) {
    const MotorPins& motor = kMotors[i];
    EncoderState& encoder = gEncoderStates[i];
    WheelControlState& wheel = gWheelControl[i];

    if (std::fabs(wheel.targetNorm) < kDeadband) {
      wheel.targetNorm = 0.0f;
      wheel.targetRpm = 0.0f;
      wheel.measuredRpm = 0.0f;
      wheel.integrator = 0.0f;
      wheel.output = 0.0f;
      stopMotor(motor);
      continue;
    }

    if (!encoder.enabled) {
      wheel.output = clampUnit(wheel.targetNorm);
      setMotorOutput(motor, wheel.output);
      continue;
    }

    noInterrupts();
    const int32_t currentCount = encoder.count;
    interrupts();

    const int32_t delta = currentCount - encoder.controlLastCount;
    encoder.controlLastCount = currentCount;

    wheel.measuredRpm =
        (static_cast<float>(delta) * 60.0f) /
        (static_cast<float>(kEncoderCountsPerRevolution) * dtSeconds);

    const float error = wheel.targetRpm - wheel.measuredRpm;
    wheel.integrator = clampUnit(
        wheel.integrator + (kSpeedKi * error * dtSeconds) / kIntegratorLimit) *
                      kIntegratorLimit;

    float output =
        (kFeedforwardGain * wheel.targetNorm) + (kSpeedKp * error) + wheel.integrator;
    output = clampUnit(output);

    wheel.output = output;
    setMotorOutput(motor, wheel.output);
  }
}

void runEncoderTestMode() {
  // Direct motor control without closed-loop
  // Left wheels backward (0, 2), Right wheels forward (1, 3)
  setMotorOutput(kMotors[0], -1.0f);  // FL backward
  setMotorOutput(kMotors[1], 1.0f);   // FR forward
  setMotorOutput(kMotors[2], -1.0f);  // RL backward
  setMotorOutput(kMotors[3], 1.0f);   // RR forward
}

bool parseDriveCommand(const std::string& payload, DriveCommand& command) {
  if (payload == "STOP") {
    command = DriveCommand{};
    return true;
  }

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

  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(omega)) {
    return false;
  }

  command.x = clampUnit(x);
  command.y = clampUnit(y);
  command.omega = clampUnit(omega);
  return true;
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    if (!kSerialOnlyReceivedValues) {
      Serial.printf("BLE connected, active links: %d\n", server->getConnectedCount());
    }
  }

  void onDisconnect(NimBLEServer* server) override {
    stopAllMotors();
    gDriveCommand = DriveCommand{};
    NimBLEDevice::startAdvertising();
    if (!kSerialOnlyReceivedValues) {
      Serial.printf("BLE disconnected, active links: %d\n", server->getConnectedCount());
    }
  }
};

class ControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    if (kEncoderTestOnly) {
      return;
    }

    const std::string payload = characteristic->getValue();
    DriveCommand nextCommand;

    if (!parseDriveCommand(payload, nextCommand)) {
      if (!kSerialOnlyReceivedValues) {
        Serial.printf("BLE RX bad payload: %s\n", payload.c_str());
      }
      return;
    }

    gDriveCommand = nextCommand;
    gLastCommandMs = millis();
    applyDriveCommand(gDriveCommand);
    Serial.printf("%.3f,%.3f,%.3f\n", gDriveCommand.x, gDriveCommand.y,
                  gDriveCommand.omega);
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
    if (!kSerialOnlyReceivedValues) {
      Serial.printf(
          "Status LED disabled: RGB pin %u conflicts with encoder pin mapping\n",
          static_cast<unsigned>(kRgbPin));
    }
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

  gEncoderTestStartMs = millis();
  gLastEncoderTestPhase = -1;
  gLastEncoderReportMs = millis();

  if (kEncoderTestOnly) {
    if (!kSerialOnlyReceivedValues) {
      Serial.println("Encoder test mode enabled");
      Serial.println("Sequence: each wheel forward/backward for 2.5s");
      Serial.println("Watch logs: ENC FL/FR/RL/RR should match active wheel");
    }
  }

  NimBLEDevice::init(kBleDeviceName);
  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(&gServerCallbacks);

  NimBLEService* service = gServer->createService(kServiceUuid);
    gControlCharacteristic = service->createCharacteristic(
      kControlCharUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    gTelemetryCharacteristic = service->createCharacteristic(
      kTelemetryCharUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    gControlCharacteristic->setCallbacks(&gControlCallbacks);
    gControlCharacteristic->setValue("ready");
    gTelemetryCharacteristic->setValue("T,0,0,0,0,0,0,0,0");
  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(kBleDeviceName);
  advertising->addServiceUUID(kServiceUuid);
  advertising->start();

  gLastCommandMs = millis();
  gSequenceStartMs = millis();
  gSequenceFinished = false;
  gLastEncoderReportMs = millis();
  gLastControlMs = millis();
  gLastTelemetryMs = millis();

  if (!kSerialOnlyReceivedValues) {
    Serial.println("BLE DRIVE controller started");
    Serial.printf("Advertising as: %s\n", kBleDeviceName);
    Serial.println("Payload format: DRIVE:x,y,omega");
    Serial.println("Telemetry notify UUID: 12345678-1234-1234-1234-1234567890ad");
    Serial.println("Telemetry frame: T,fl_set,fl_meas,fr_set,fr_meas,rl_set,rl_meas,rr_set,rr_meas");
    Serial.println("Serial Plotter: fl_set,fl_meas,fr_set,fr_meas,rl_set,rl_meas,rr_set,rr_meas");
  }
}

void loop() {
  updateHeartbeat();
  reportEncoders();

  if (kEncoderTestOnly) {
    runEncoderTestMode();
    sendBleTelemetry();
    delay(20);
    return;
  }

  DriveCommand commandForLoop = gDriveCommand;

  if (kRunStartupDriveSequence) {
    const uint32_t elapsed = millis() - gSequenceStartMs;
    if (elapsed < kTestPhaseDurationMs) {
      commandForLoop.x = 0.0f;
      commandForLoop.y = kTestMoveSpeed;
      commandForLoop.omega = 0.0f;
    } else if (elapsed < (2 * kTestPhaseDurationMs)) {
      commandForLoop.x = 0.0f;
      commandForLoop.y = -kTestMoveSpeed;
      commandForLoop.omega = 0.0f;
    } else if (elapsed < (3 * kTestPhaseDurationMs)) {
      commandForLoop.x = kTestMoveSpeed;
      commandForLoop.y = 0.0f;
      commandForLoop.omega = 0.0f;
    } else if (elapsed < (4 * kTestPhaseDurationMs)) {
      commandForLoop.x = -kTestMoveSpeed;
      commandForLoop.y = 0.0f;
      commandForLoop.omega = 0.0f;
    } else if (!gSequenceFinished) {
      gSequenceFinished = true;
    }
  }

  if (millis() - gLastCommandMs > kCommandTimeoutMs) {
    gDriveCommand = DriveCommand{};
    commandForLoop = gDriveCommand;
  } else {
    commandForLoop = gDriveCommand;
  }

  applyDriveCommand(commandForLoop);
  updateClosedLoopControl();
  sendBleTelemetry();

  delay(20);
}
