#include <Arduino.h>

#include <cctype>

namespace {
constexpr int kLidarPwmPin = 14;
constexpr int kLidarRxPin = 13;
constexpr int kDfPlayerRxPin = 11;  // ESP RX, podlacz do TX DFPlayer
constexpr int kDfPlayerTxPin = 12;  // ESP TX, podlacz do RX DFPlayer

constexpr int kPwmChannel = 0;
constexpr int kPwmFrequencyHz = 20000;
constexpr int kPwmResolutionBits = 8;
constexpr int kPwmDuty = 180;  // 0..255

constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kDfPlayerBaud = 9600;
constexpr uint32_t kStatsIntervalMs = 1000;
constexpr uint32_t kBaudSwitchIntervalMs = 2500;
constexpr uint32_t kDfPlayerInitDelayMs = 500;
constexpr int kHexDumpLineBytes = 24;
constexpr size_t kRawPrintBytes = 1200;
constexpr size_t kSampleBytes = 2048;
constexpr uint8_t kMinFrameLen = 8;
constexpr uint8_t kMaxFrameLen = 96;
constexpr int kTopHeaders = 8;
constexpr uint8_t kDfPlayerDefaultVolume = 24;
constexpr size_t kConsoleLineMaxLen = 64;
constexpr bool kDfPlayerUseFeedback = true;
constexpr uint32_t kConsoleCommandTimeoutMs = 150;

constexpr uint32_t kBaudCandidates[] = {
  230400,
  115200,
  256000,
  128000,
  460800,
};
constexpr size_t kBaudCandidateCount =
  sizeof(kBaudCandidates) / sizeof(kBaudCandidates[0]);

enum class ChecksumType : uint8_t {
  None,
  Sum8,
  Xor8,
  Crc8_07,
  Crc8_31,
  Sum16,
  Xor16,
  Crc16_1021,
  Crc16_A001,
};

struct FrameModel {
  bool valid = false;
  uint8_t header = 0;
  uint8_t header2 = 0;
  bool twoByteHeader = false;
  uint8_t length = 0;
  ChecksumType checksum = ChecksumType::Sum8;
  float score = 0.0f;
  int tested = 0;
  int passed = 0;
};

uint32_t gRxBytes = 0;
uint32_t gValidFrames = 0;
uint32_t gBadFrames = 0;
uint32_t gLastStatsMs = 0;
int gHexLineCount = 0;
size_t gSampleCount = 0;
uint8_t gSample[kSampleBytes];
bool gAnalysisDone = false;
FrameModel gModel;

size_t gBaudIndex = 0;
uint32_t gCurrentBaud = kBaudCandidates[0];
uint32_t gLastBaudSwitchMs = 0;

bool gFrameSync = false;
uint8_t gFrameIndex = 0;
uint8_t gFrameBuf[kMaxFrameLen];
bool gPointCloudMode = false;
String gConsoleLine;
uint8_t gDfPlayerVolume = kDfPlayerDefaultVolume;
int gDfPlayerRxDumpCount = 0;
uint32_t gLastConsoleInputMs = 0;

constexpr uint8_t kLd06Header1 = 0x54;
constexpr uint8_t kLd06Header2 = 0x2C;
constexpr uint8_t kLd06FrameLen = 47;
constexpr int kLd06PointCount = 12;

HardwareSerial& gLidarSerial = Serial2;
HardwareSerial& gDfPlayerSerial = Serial1;

void printHexByte(uint8_t b);

uint16_t dfPlayerChecksum(uint8_t command, uint16_t parameter) {
  const uint16_t sum = static_cast<uint16_t>(0xFF + 0x06 + command + 0x00 +
                                             (parameter >> 8) +
                                             (parameter & 0xFF));
  return static_cast<uint16_t>(0U - sum);
}

void sendDfPlayerCommand(uint8_t command, uint16_t parameter) {
  const uint16_t checksum = dfPlayerChecksum(command, parameter);
  const uint8_t frame[] = {
      0x7E,
      0xFF,
      0x06,
      command,
  static_cast<uint8_t>(kDfPlayerUseFeedback ? 0x01 : 0x00),
      static_cast<uint8_t>((parameter >> 8) & 0xFF),
      static_cast<uint8_t>(parameter & 0xFF),
      static_cast<uint8_t>((checksum >> 8) & 0xFF),
      static_cast<uint8_t>(checksum & 0xFF),
      0xEF,
  };
  gDfPlayerSerial.write(frame, sizeof(frame));
}

void setDfPlayerVolume(uint8_t volume) {
  gDfPlayerVolume = static_cast<uint8_t>(constrain(volume, 0, 30));
  sendDfPlayerCommand(0x06, gDfPlayerVolume);
}

void initDfPlayer() {
  gDfPlayerSerial.begin(kDfPlayerBaud, SERIAL_8N1, kDfPlayerRxPin,
                        kDfPlayerTxPin);
  delay(kDfPlayerInitDelayMs);
  sendDfPlayerCommand(0x0C, 0);
  delay(1000);
  sendDfPlayerCommand(0x09, 0x0002);
  delay(300);
  setDfPlayerVolume(kDfPlayerDefaultVolume);
  delay(100);
  sendDfPlayerCommand(0x16, 0);
}

void printDfPlayerHelp() {
  Serial.println("MP3 commands:");
  Serial.println("  MP3 PLAY <track>");
  Serial.println("  MP3 LOOP <track>");
  Serial.println("  MP3 STOP");
  Serial.println("  MP3 NEXT");
  Serial.println("  MP3 PREV");
  Serial.println("  MP3 VOL <0-30>");
  Serial.println("  MP3 INIT");
  Serial.println("  MP3 HELP");
}

void trimAsciiWhitespace(String& text) {
  while (text.length() > 0 &&
         std::isspace(static_cast<unsigned char>(text[0])) != 0) {
    text.remove(0, 1);
  }
  while (text.length() > 0 &&
         std::isspace(static_cast<unsigned char>(text[text.length() - 1])) != 0) {
    text.remove(text.length() - 1, 1);
  }
}

void handleConsoleCommand(String line) {
  trimAsciiWhitespace(line);
  if (line.length() == 0) {
    return;
  }

  line.toUpperCase();
  if (!line.startsWith("MP3")) {
    Serial.println("[cmd] unknown. Use MP3 HELP");
    return;
  }

  String args = line.substring(3);
  trimAsciiWhitespace(args);

  if (args == "HELP") {
    printDfPlayerHelp();
    return;
  }

  if (args == "STOP") {
    sendDfPlayerCommand(0x16, 0);
    Serial.println("[mp3] stop");
    return;
  }

  if (args == "NEXT") {
    sendDfPlayerCommand(0x01, 0);
    Serial.println("[mp3] next");
    return;
  }

  if (args == "PREV") {
    sendDfPlayerCommand(0x02, 0);
    Serial.println("[mp3] prev");
    return;
  }

  if (args == "INIT") {
    initDfPlayer();
    Serial.println("[mp3] reinitialized");
    return;
  }

  if (args.startsWith("PLAY ")) {
    const long track = args.substring(5).toInt();
    if (track <= 0 || track > 2999) {
      Serial.println("[mp3] invalid track");
      return;
    }
    sendDfPlayerCommand(0x03, static_cast<uint16_t>(track));
    Serial.printf("[mp3] play track=%ld\n", track);
    return;
  }

  if (args.startsWith("LOOP ")) {
    const long track = args.substring(5).toInt();
    if (track <= 0 || track > 2999) {
      Serial.println("[mp3] invalid track");
      return;
    }
    sendDfPlayerCommand(0x08, static_cast<uint16_t>(track));
    Serial.printf("[mp3] loop track=%ld\n", track);
    return;
  }

  if (args.startsWith("VOL ")) {
    const long volume = args.substring(4).toInt();
    if (volume < 0 || volume > 30) {
      Serial.println("[mp3] invalid volume");
      return;
    }
    setDfPlayerVolume(static_cast<uint8_t>(volume));
    Serial.printf("[mp3] volume=%ld\n", volume);
    return;
  }

  Serial.println("[cmd] unknown. Use MP3 HELP");
}

void processUsbConsole() {
  while (Serial.available() > 0) {
    const int raw = Serial.read();
    if (raw < 0) {
      return;
    }

    const char ch = static_cast<char>(raw);
    if (ch == '\r') {
      continue;
    }

    gLastConsoleInputMs = millis();

    if (ch == '\n') {
      handleConsoleCommand(gConsoleLine);
      gConsoleLine = "";
      continue;
    }

    if (gConsoleLine.length() < kConsoleLineMaxLen) {
      gConsoleLine += ch;
    }
  }

  if (gConsoleLine.length() > 0 &&
      millis() - gLastConsoleInputMs >= kConsoleCommandTimeoutMs) {
    handleConsoleCommand(gConsoleLine);
    gConsoleLine = "";
  }
}

void processDfPlayerResponses() {
  while (gDfPlayerSerial.available() > 0) {
    const int raw = gDfPlayerSerial.read();
    if (raw < 0) {
      return;
    }

    if (gDfPlayerRxDumpCount == 0) {
      Serial.print("[mp3-rx] ");
    }

    const uint8_t b = static_cast<uint8_t>(raw);
    printHexByte(b);
    ++gDfPlayerRxDumpCount;

    if (gDfPlayerRxDumpCount >= 10) {
      Serial.println();
      gDfPlayerRxDumpCount = 0;
    }
  }
}

const char* checksumName(ChecksumType type) {
  switch (type) {
    case ChecksumType::None:
      return "none";
    case ChecksumType::Sum8:
      return "sum8";
    case ChecksumType::Xor8:
      return "xor8";
    case ChecksumType::Crc8_07:
      return "crc8_07";
    case ChecksumType::Crc8_31:
      return "crc8_31";
    case ChecksumType::Sum16:
      return "sum16";
    case ChecksumType::Xor16:
      return "xor16";
    case ChecksumType::Crc16_1021:
      return "crc16_1021";
    case ChecksumType::Crc16_A001:
      return "crc16_a001";
  }
  return "unknown";
}

void printHexByte(uint8_t b) {
  const char* hex = "0123456789ABCDEF";
  Serial.print(hex[(b >> 4) & 0x0F]);
  Serial.print(hex[b & 0x0F]);
  Serial.print(' ');
}

uint8_t crc8Poly(const uint8_t* data, size_t len, uint8_t poly, uint8_t init) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ poly)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

uint16_t crc16Poly(const uint8_t* data,
                   size_t len,
                   uint16_t poly,
                   uint16_t init,
                   bool msbFirst) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    if (msbFirst) {
      crc ^= static_cast<uint16_t>(data[i]) << 8;
      for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ poly)
                             : static_cast<uint16_t>(crc << 1);
      }
    } else {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b) {
        crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ poly)
                             : static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

uint16_t computeChecksum(const uint8_t* data,
                         size_t lenWithoutChecksum,
                         ChecksumType type) {
  uint16_t acc = 0;

  switch (type) {
    case ChecksumType::None:
      return 0;
    case ChecksumType::Sum8:
      for (size_t i = 0; i < lenWithoutChecksum; ++i) {
        acc = static_cast<uint8_t>(acc + data[i]);
      }
      return acc;
    case ChecksumType::Xor8:
      for (size_t i = 0; i < lenWithoutChecksum; ++i) {
        acc ^= data[i];
      }
      return acc;
    case ChecksumType::Crc8_07:
      return crc8Poly(data, lenWithoutChecksum, 0x07, 0x00);
    case ChecksumType::Crc8_31:
      return crc8Poly(data, lenWithoutChecksum, 0x31, 0x00);
    case ChecksumType::Sum16:
      for (size_t i = 0; i < lenWithoutChecksum; ++i) {
        acc = static_cast<uint16_t>(acc + data[i]);
      }
      return acc;
    case ChecksumType::Xor16:
      for (size_t i = 0; i < lenWithoutChecksum; ++i) {
        acc ^= data[i];
      }
      return acc;
    case ChecksumType::Crc16_1021:
      return crc16Poly(data, lenWithoutChecksum, 0x1021, 0x0000, true);
    case ChecksumType::Crc16_A001:
      return crc16Poly(data, lenWithoutChecksum, 0xA001, 0xFFFF, false);
  }

  return 0;
}

uint8_t checksumBytes(ChecksumType type) {
  switch (type) {
    case ChecksumType::None:
      return 0;
    case ChecksumType::Sum8:
    case ChecksumType::Xor8:
    case ChecksumType::Crc8_07:
    case ChecksumType::Crc8_31:
      return 1;
    case ChecksumType::Sum16:
    case ChecksumType::Xor16:
    case ChecksumType::Crc16_1021:
    case ChecksumType::Crc16_A001:
      return 2;
  }
  return 1;
}

bool validateFrame(const uint8_t* frame, size_t len, ChecksumType type) {
  if (type == ChecksumType::None) {
    return len > 0;
  }

  const uint8_t csBytes = checksumBytes(type);
  if (len < static_cast<size_t>(1 + csBytes)) {
    return false;
  }

  const uint16_t expected = computeChecksum(frame, len - csBytes, type);
  if (csBytes == 1) {
    return static_cast<uint8_t>(expected & 0xFF) == frame[len - 1];
  }

  const uint16_t gotLE = static_cast<uint16_t>(frame[len - 2]) |
                         (static_cast<uint16_t>(frame[len - 1]) << 8);
  const uint16_t gotBE = static_cast<uint16_t>(frame[len - 1]) |
                         (static_cast<uint16_t>(frame[len - 2]) << 8);
  return expected == gotLE || expected == gotBE;
}

void printFrameBrief(const uint8_t* frame, size_t len) {
  Serial.print("frame ok: ");
  const size_t shown = (len < 16) ? len : 16;
  for (size_t i = 0; i < shown; ++i) {
    printHexByte(frame[i]);
  }
  if (len > shown) {
    Serial.print("...");
  }
  Serial.println();
}

float normalizeAngle(float angleDeg) {
  while (angleDeg < 0.0f) {
    angleDeg += 360.0f;
  }
  while (angleDeg >= 360.0f) {
    angleDeg -= 360.0f;
  }
  return angleDeg;
}

void printPointCloudLd06(const uint8_t* frame) {
  const uint16_t speed = static_cast<uint16_t>(frame[2]) |
                         (static_cast<uint16_t>(frame[3]) << 8);
  const float startAngle =
      (static_cast<uint16_t>(frame[4]) |
       (static_cast<uint16_t>(frame[5]) << 8)) /
      100.0f;
  const float endAngle =
      (static_cast<uint16_t>(frame[42]) |
       (static_cast<uint16_t>(frame[43]) << 8)) /
      100.0f;

  float span = endAngle - startAngle;
  if (span < 0.0f) {
    span += 360.0f;
  }
  const float step = span / static_cast<float>(kLd06PointCount - 1);

  // CSV-like output for easy plotting/parsing over USB serial.
  for (int i = 0; i < kLd06PointCount; ++i) {
    const int base = 6 + i * 3;
    const uint16_t distanceMm = static_cast<uint16_t>(frame[base]) |
                                (static_cast<uint16_t>(frame[base + 1]) << 8);
    const uint8_t confidence = frame[base + 2];
    const float angle = normalizeAngle(startAngle + step * i);

    Serial.print("PT,");
    Serial.print(angle, 2);
    Serial.print(',');
    Serial.print(distanceMm);
    Serial.print(',');
    Serial.print(confidence);
    Serial.print(',');
    Serial.println(speed);
  }
}

void onValidFrame(const uint8_t* frame, size_t len) {
  ++gValidFrames;

  if (gPointCloudMode) {
    printPointCloudLd06(frame);
    return;
  }

  printFrameBrief(frame, len);
}

void resetAnalysisState() {
  gRxBytes = 0;
  gValidFrames = 0;
  gBadFrames = 0;
  gHexLineCount = 0;
  gSampleCount = 0;
  gAnalysisDone = false;
  gModel = FrameModel{};
  gFrameSync = false;
  gFrameIndex = 0;
}

void startLidarSerial(uint32_t baud) {
  gLidarSerial.end();
  delay(20);
  gLidarSerial.begin(baud, SERIAL_8N1, kLidarRxPin, -1);
}

void insertTopHeader(uint8_t header,
                     int freq,
                     uint8_t headers[kTopHeaders],
                     int freqs[kTopHeaders]) {
  for (int i = 0; i < kTopHeaders; ++i) {
    if (freq > freqs[i]) {
      for (int j = kTopHeaders - 1; j > i; --j) {
        freqs[j] = freqs[j - 1];
        headers[j] = headers[j - 1];
      }
      freqs[i] = freq;
      headers[i] = header;
      return;
    }
  }
}

FrameModel autoDetectModel() {
  FrameModel best;
  if (gSampleCount < 300) {
    return best;
  }

  int headerFreq[256] = {0};
  for (size_t i = 0; i < gSampleCount; ++i) {
    ++headerFreq[gSample[i]];
  }

  uint8_t topHeaders[kTopHeaders] = {0};
  int topFreqs[kTopHeaders] = {0};
  for (int h = 0; h < 256; ++h) {
    if (headerFreq[h] >= 8) {
      insertTopHeader(static_cast<uint8_t>(h), headerFreq[h], topHeaders,
                      topFreqs);
    }
  }

  const ChecksumType checksumCandidates[] = {
      ChecksumType::Sum8,
      ChecksumType::Xor8,
      ChecksumType::Crc8_07,
      ChecksumType::Crc8_31,
      ChecksumType::Sum16,
      ChecksumType::Xor16,
      ChecksumType::Crc16_1021,
      ChecksumType::Crc16_A001,
  };

  for (int h = 0; h < kTopHeaders; ++h) {
    if (topFreqs[h] == 0) {
      continue;
    }

    const uint8_t header = topHeaders[h];
    for (uint8_t len = kMinFrameLen; len <= kMaxFrameLen; ++len) {
      for (ChecksumType ck : checksumCandidates) {
        int tested = 0;
        int passed = 0;

        for (size_t i = 0; i + len <= gSampleCount; ++i) {
          if (gSample[i] != header) {
            continue;
          }
          ++tested;
          if (validateFrame(&gSample[i], len, ck)) {
            ++passed;
          }
        }

        if (tested < 8) {
          continue;
        }

        const float score = static_cast<float>(passed) / tested;
        if (!best.valid || score > best.score ||
            (score == best.score && tested > best.tested)) {
          best.valid = true;
          best.header = header;
          best.header2 = 0;
          best.twoByteHeader = false;
          best.length = len;
          best.checksum = ck;
          best.score = score;
          best.tested = tested;
          best.passed = passed;
        }
      }
    }
  }

  if (best.valid && best.score < 0.60f) {
    best.valid = false;
  }

  if (!best.valid) {
    // Fallback for protocols with 2-byte sync word and no simple checksum.
    static uint16_t pairFreq[256][256];
    for (int a = 0; a < 256; ++a) {
      for (int b = 0; b < 256; ++b) {
        pairFreq[a][b] = 0;
      }
    }
    for (size_t i = 0; i + 1 < gSampleCount; ++i) {
      ++pairFreq[gSample[i]][gSample[i + 1]];
    }

    float bestPeriodicScore = 0.0f;
    uint8_t bestH1 = 0;
    uint8_t bestH2 = 0;
    int bestPairHits = 0;
    int bestDist = 0;
    int bestDistHits = 0;

    for (int a = 0; a < 256; ++a) {
      for (int b = 0; b < 256; ++b) {
        const int hits = pairFreq[a][b];
        if (hits < 8) {
          continue;
        }
        if (a == 0x00 && b == 0x00) {
          continue;
        }

        int positions[256] = {0};
        int posCount = 0;
        for (size_t i = 0; i + 1 < gSampleCount && posCount < 256; ++i) {
          if (gSample[i] == static_cast<uint8_t>(a) &&
              gSample[i + 1] == static_cast<uint8_t>(b)) {
            positions[posCount++] = static_cast<int>(i);
          }
        }

        int distHist[kMaxFrameLen + 1] = {0};
        for (int i = 1; i < posCount; ++i) {
          const int d = positions[i] - positions[i - 1];
          if (d >= kMinFrameLen && d <= kMaxFrameLen) {
            ++distHist[d];
          }
        }

        int localBestDist = 0;
        int localBestDistHits = 0;
        for (int d = kMinFrameLen; d <= kMaxFrameLen; ++d) {
          if (distHist[d] > localBestDistHits) {
            localBestDistHits = distHist[d];
            localBestDist = d;
          }
        }

        if (localBestDistHits < 4) {
          continue;
        }

        const float periodicScore = static_cast<float>(localBestDistHits) / hits;
        if (periodicScore > bestPeriodicScore ||
            (periodicScore == bestPeriodicScore && hits > bestPairHits)) {
          bestPeriodicScore = periodicScore;
          bestH1 = static_cast<uint8_t>(a);
          bestH2 = static_cast<uint8_t>(b);
          bestPairHits = hits;
          bestDist = localBestDist;
          bestDistHits = localBestDistHits;
        }
      }
    }

    if (bestDistHits >= 4 && bestPeriodicScore >= 0.75f) {
      best.valid = true;
      best.header = bestH1;
      best.header2 = bestH2;
      best.twoByteHeader = true;
      best.length = static_cast<uint8_t>(bestDist);
      best.checksum = ChecksumType::None;
      best.tested = bestPairHits;
      best.passed = bestDistHits;
      best.score = bestPeriodicScore;
    }
  }

  return best;
}

void processByteWithModel(uint8_t b) {
  if (!gModel.valid) {
    return;
  }

  if (gModel.twoByteHeader) {
    if (!gFrameSync) {
      if (gFrameIndex == 0) {
        if (b == gModel.header) {
          gFrameBuf[0] = b;
          gFrameIndex = 1;
        }
        return;
      }

      if (gFrameIndex == 1) {
        if (b == gModel.header2) {
          gFrameBuf[1] = b;
          gFrameIndex = 2;
          gFrameSync = true;
        } else if (b == gModel.header) {
          gFrameBuf[0] = b;
          gFrameIndex = 1;
        } else {
          gFrameIndex = 0;
        }
        return;
      }
    }

    gFrameBuf[gFrameIndex++] = b;
    if (gFrameIndex < gModel.length) {
      return;
    }

    onValidFrame(gFrameBuf, gModel.length);
    gFrameSync = false;
    gFrameIndex = 0;
    return;
  }

  if (!gFrameSync) {
    if (b == gModel.header) {
      gFrameSync = true;
      gFrameIndex = 0;
      gFrameBuf[gFrameIndex++] = b;
    }
    return;
  }

  gFrameBuf[gFrameIndex++] = b;
  if (gFrameIndex < gModel.length) {
    return;
  }

  if (validateFrame(gFrameBuf, gModel.length, gModel.checksum)) {
    onValidFrame(gFrameBuf, gModel.length);
    gFrameSync = false;
    gFrameIndex = 0;
    return;
  }

  ++gBadFrames;
  gFrameSync = false;
  gFrameIndex = 0;
  if (b == gModel.header) {
    gFrameSync = true;
    gFrameBuf[gFrameIndex++] = b;
  }
}
}

void setup() {
  Serial.begin(kUsbBaud);
  const uint32_t startMs = millis();
  while (!Serial && millis() - startMs < 3000) {
    delay(10);
  }

  ledcSetup(kPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttachPin(kLidarPwmPin, kPwmChannel);
  ledcWrite(kPwmChannel, kPwmDuty);

  initDfPlayer();

  gCurrentBaud = kBaudCandidates[gBaudIndex];
  startLidarSerial(gCurrentBaud);
  gLastBaudSwitchMs = millis();

  Serial.println();
  Serial.println("LiDAR start");
  Serial.println("PWM: pin=14, ch=0, freq=20kHz, duty=180/255");
  Serial.println("UART: RX=13, TX=none, baud=auto-scan");
  Serial.println("DFPlayer: RX=12, TX=11, baud=9600");
  Serial.println("DFPlayer source: TF card");
  Serial.println("Mode: RAW capture + auto frame detect");
  printDfPlayerHelp();
  Serial.print("[baud] start=");
  Serial.println(gCurrentBaud);
}

void loop() {
  processUsbConsole();
  processDfPlayerResponses();

  while (gLidarSerial.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(gLidarSerial.read());
    ++gRxBytes;

    if (gSampleCount < kSampleBytes) {
      gSample[gSampleCount++] = b;
    }

    if (!gAnalysisDone && gRxBytes <= kRawPrintBytes) {
      // Print early raw bytes so unknown protocol traffic is visible.
      printHexByte(b);
      ++gHexLineCount;
      if (gHexLineCount >= kHexDumpLineBytes) {
        Serial.println();
        gHexLineCount = 0;
      }
    }

    processByteWithModel(b);
  }

  if (!gAnalysisDone && gSampleCount >= kSampleBytes) {
    const uint32_t t0 = millis();
    gModel = autoDetectModel();
    const uint32_t t1 = millis();
    gAnalysisDone = true;

    Serial.println();
    Serial.println("[analysis] sample complete");
    Serial.print("[analysis] bytes=");
    Serial.print(gSampleCount);
    Serial.print(" time_ms=");
    Serial.println(t1 - t0);

    if (gModel.valid) {
      gPointCloudMode =
          (gModel.length == kLd06FrameLen) &&
          ((gModel.twoByteHeader && gModel.header == kLd06Header1 &&
            gModel.header2 == kLd06Header2) ||
           (!gModel.twoByteHeader && gModel.header == kLd06Header1));

      Serial.print("[analysis] header=0x");
      printHexByte(gModel.header);
      if (gModel.twoByteHeader) {
        Serial.print(" hdr2=0x");
        printHexByte(gModel.header2);
      }
      Serial.print(" len=");
      Serial.print(gModel.length);
      Serial.print(" checksum=");
      Serial.print(checksumName(gModel.checksum));
      Serial.print(" score=");
      Serial.print(gModel.score, 3);
      Serial.print(" passed/tested=");
      Serial.print(gModel.passed);
      Serial.print('/');
      Serial.println(gModel.tested);
      if (gPointCloudMode) {
        Serial.println("[analysis] point_cloud=enabled format=LD06-like");
      }
    } else {
      Serial.println("[analysis] no reliable frame model found");
    }
  }

  if (gAnalysisDone && !gModel.valid) {
    const uint32_t now = millis();
    if (now - gLastBaudSwitchMs >= kBaudSwitchIntervalMs) {
      gBaudIndex = (gBaudIndex + 1) % kBaudCandidateCount;
      gCurrentBaud = kBaudCandidates[gBaudIndex];

      Serial.print("[baud] switching to ");
      Serial.println(gCurrentBaud);

      startLidarSerial(gCurrentBaud);
      resetAnalysisState();
      gLastBaudSwitchMs = now;
    }
  }

  const uint32_t now = millis();
  if (now - gLastStatsMs >= kStatsIntervalMs) {
    gLastStatsMs = now;
    Serial.println();
    Serial.print("[stats] rx_bytes=");
    Serial.print(gRxBytes);
    Serial.print(" sample=");
    Serial.print(gSampleCount);
    Serial.print(" baud=");
    Serial.print(gCurrentBaud);
    Serial.print(" frames_ok=");
    Serial.print(gValidFrames);
    Serial.print(" frames_bad=");
    Serial.print(gBadFrames);

    if (!gAnalysisDone) {
      Serial.print(" analysis=pending");
    } else if (!gModel.valid) {
      Serial.print(" analysis=no_model");
    } else {
      Serial.print(" analysis=ok hdr=0x");
      printHexByte(gModel.header);
      if (gModel.twoByteHeader) {
        Serial.print(" hdr2=0x");
        printHexByte(gModel.header2);
      }
      Serial.print(" len=");
      Serial.print(gModel.length);
      Serial.print(" ck=");
      Serial.print(checksumName(gModel.checksum));
      Serial.print(" score=");
      Serial.print(gModel.score, 3);
      Serial.print(" point_cloud=");
      Serial.print(gPointCloudMode ? "on" : "off");
    }

    Serial.println();
  }
}
