// 지상국 펌웨어. ESP32 + E22-900T30D-V2 만 있으면 된다(빵판 구성).
//
//   pio run -e ground -t upload -t monitor
//
// 하는 일
//   - 1단/2단 텔레메트리 프레임을 받아 한 줄씩 사람이 읽는 형식으로 출력
//   - USB 시리얼로 ARM / DISARM / PING 명령을 받아 무선으로 중계
//   - 각 노드의 마지막 수신 시각, 수신 프레임 수, CRC 오류 수 표시
//
// 화약을 터뜨리는 명령은 없다. 프로토콜 자체에 존재하지 않는다.
//
// 핀은 빵판 배선에 맞춰 아래 상수를 고친다. 비행 보드의 board_pins.h 와는
// 별개다.

#include <Arduino.h>
#include <HardwareSerial.h>

#include "flight_types.h"
#include "telemetry_protocol.h"

namespace {

// ── 빵판 배선. 여기만 고치면 된다 ──────────────────────────────
constexpr int kRadioRxPin = 16;  // ESP32 RX  <- E22 TXD
constexpr int kRadioTxPin = 17;  // ESP32 TX  -> E22 RXD
constexpr int kRadioM0Pin = 21;
constexpr int kRadioM1Pin = 22;
constexpr int kRadioAuxPin = 23;
constexpr uint32_t kRadioBaud = 9600;
constexpr uint32_t kConsoleBaud = 115200;

HardwareSerial radio(1);
telemetry::FrameReader reader;

struct NodeStatus {
  uint32_t frames = 0;
  uint32_t lastSeenMs = 0;
  bool seen = false;
};
NodeStatus stage1;
NodeStatus stage2;

uint16_t nextNonce = 1;
String consoleLine;

const char* stateName(uint8_t value) {
  return flightStateName(static_cast<FlightState>(value));
}
const char* reasonName(uint8_t value) {
  return reasonCodeName(static_cast<ReasonCode>(value));
}

void printTelemetry(telemetry::NodeId source,
                    const telemetry::TelemetryPayload& p) {
  using namespace telemetry;
  const char* who = source == NodeId::Stage1 ? "S1" : "S2";
  // 상태 비트를 한눈에 보이게 깐다.
  char flags[7];
  flags[0] = (p.flags & StatusFlag::kArmed) ? 'A' : '-';
  flags[1] = (p.flags & StatusFlag::kHeaterArmed) ? 'H' : '-';
  flags[2] = (p.flags & StatusFlag::kSeparated) ? 'S' : '-';
  flags[3] = (p.flags & StatusFlag::kImuValid) ? 'I' : '-';
  flags[4] = (p.flags & StatusFlag::kBarometerValid) ? 'B' : '-';
  flags[5] = (p.flags & StatusFlag::kStageLinkPresentAtBoot) ? 'L' : '-';
  flags[6] = '\0';

  Serial.printf(
      "%s %8lu %-10s %-18s [%s] alt=%7.1fm vz=%6.1f a=%5.2fg max=%6.1fm "
      "tilt=%5.1f batt=%5.2fV drop=%u\n",
      who, static_cast<unsigned long>(p.timestampMs), stateName(p.state),
      reasonName(p.reason), flags, static_cast<double>(p.altitudeM),
      p.verticalVelocityCmps / 100.0, p.accelerationMilliG / 100.0,
      p.maxAltitudeDm / 10.0, p.tiltDeciDeg / 10.0, p.batteryMilliV / 1000.0,
      static_cast<unsigned>(p.droppedFrames));
}

void sendCommand(telemetry::RemoteCommand command, uint8_t target) {
  using namespace telemetry;
  CommandPayload payload;
  payload.command = static_cast<uint8_t>(command);
  payload.target = target;
  payload.nonce = nextNonce++;

  uint8_t buffer[kMaxFrameBytes];
  const size_t length =
      encodeCommand(NodeId::Ground, payload, buffer, sizeof(buffer));
  if (length == 0) return;

  // 투명 모드에서는 공중 충돌을 피할 방법이 없으니 같은 명령을 세 번
  // 보낸다. 수신 측은 nonce 로 중복을 걸러낸다.
  for (int i = 0; i < 3; ++i) {
    while (digitalRead(kRadioAuxPin) != HIGH) delay(2);
    radio.write(buffer, length);
    radio.flush();
    delay(60);
  }
  Serial.printf("[TX] command=%u target=0x%02X nonce=%u (x3)\n",
                payload.command, payload.target,
                static_cast<unsigned>(payload.nonce));
}

void printHelp() {
  Serial.println("commands:");
  Serial.println("  arm1 | arm2 | arma     ARM stage1 / stage2 / both");
  Serial.println("  dis1 | dis2 | disa     DISARM stage1 / stage2 / both");
  Serial.println("  ping                   ping both");
  Serial.println("  stat                   link status");
  Serial.println("note: there is no remote fire command, by design.");
}

void printStatus() {
  const uint32_t now = millis();
  Serial.printf("S1 frames=%lu last=%s\n",
                static_cast<unsigned long>(stage1.frames),
                stage1.seen
                    ? String((now - stage1.lastSeenMs) / 1000.0, 1).c_str()
                    : "never");
  Serial.printf("S2 frames=%lu last=%s\n",
                static_cast<unsigned long>(stage2.frames),
                stage2.seen
                    ? String((now - stage2.lastSeenMs) / 1000.0, 1).c_str()
                    : "never");
  Serial.printf("crc errors=%lu\n",
                static_cast<unsigned long>(reader.crcErrors()));
}

void handleConsoleLine(String line) {
  using namespace telemetry;
  line.trim();
  if (line.length() == 0) return;
  if (line.equalsIgnoreCase("arm1")) {
    sendCommand(RemoteCommand::Arm, static_cast<uint8_t>(NodeId::Stage1));
  } else if (line.equalsIgnoreCase("arm2")) {
    sendCommand(RemoteCommand::Arm, static_cast<uint8_t>(NodeId::Stage2));
  } else if (line.equalsIgnoreCase("arma")) {
    sendCommand(RemoteCommand::Arm, kTargetAll);
  } else if (line.equalsIgnoreCase("dis1")) {
    sendCommand(RemoteCommand::Disarm, static_cast<uint8_t>(NodeId::Stage1));
  } else if (line.equalsIgnoreCase("dis2")) {
    sendCommand(RemoteCommand::Disarm, static_cast<uint8_t>(NodeId::Stage2));
  } else if (line.equalsIgnoreCase("disa")) {
    sendCommand(RemoteCommand::Disarm, kTargetAll);
  } else if (line.equalsIgnoreCase("ping")) {
    sendCommand(RemoteCommand::Ping, kTargetAll);
  } else if (line.equalsIgnoreCase("stat")) {
    printStatus();
  } else {
    printHelp();
  }
}

}  // namespace

void setup() {
  Serial.begin(kConsoleBaud);
  delay(500);
  pinMode(kRadioM0Pin, OUTPUT);
  pinMode(kRadioM1Pin, OUTPUT);
  pinMode(kRadioAuxPin, INPUT);
  digitalWrite(kRadioM0Pin, LOW);  // 투명 전송 모드
  digitalWrite(kRadioM1Pin, LOW);
  radio.begin(kRadioBaud, SERIAL_8N1, kRadioRxPin, kRadioTxPin);

  Serial.println();
  Serial.println("NURA ground station (protocol v1)");
  printHelp();
}

void loop() {
  telemetry::ParsedFrame frame;
  for (int i = 0; i < 256 && radio.available(); ++i) {
    if (!reader.push(static_cast<uint8_t>(radio.read()), frame)) continue;

    telemetry::TelemetryPayload payload;
    if (!telemetry::decodeTelemetry(frame, payload)) continue;

    NodeStatus* node = nullptr;
    if (frame.source == telemetry::NodeId::Stage1) node = &stage1;
    if (frame.source == telemetry::NodeId::Stage2) node = &stage2;
    if (node != nullptr) {
      ++node->frames;
      node->lastSeenMs = millis();
      node->seen = true;
    }
    printTelemetry(frame.source, payload);
  }

  while (Serial.available()) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n' || value == '\r') {
      handleConsoleLine(consoleLine);
      consoleLine = "";
    } else if (consoleLine.length() < 31) {
      consoleLine += value;
    }
  }

  delay(5);
}
