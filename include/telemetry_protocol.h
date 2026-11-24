#pragma once

// NURA 2단 로켓 무선 프로토콜 v1
//
// 물리 계층: E22-900T30D-V2, 투명 전송 모드(M0=0, M1=0). 모듈은 그냥 UART
// 바이트 파이프라서 프레이밍·주소·오류검출이 전혀 없다. 그래서 이 계층에서
// 직접 만든다.
//
// 세 노드(1단, 2단, 지상국)가 같은 주파수를 공유한다. 충돌을 줄이려고
// 1단과 2단이 송신 시각을 엇갈리게 쓴다(FlightConfig 참조).
//
// 프레임 (리틀엔디안, 고정 헤더 6바이트 + 가변 페이로드 + CRC 2바이트)
//
//   off size  field
//   0   2     sync       0xA5 0x5A
//   2   1     version    1
//   3   1     source     NodeId
//   4   1     type       FrameType
//   5   1     payloadLen 페이로드 바이트 수
//   6   len   payload
//   6+len 2   crc16      bytes[2 .. 6+len-1] 에 대한 CRC16-CCITT
//
// CRC 범위에서 sync 를 뺀 이유: 수신기는 sync 를 찾아 재동기만 하고,
// 검증은 그 뒤 내용에만 하면 된다.
//
// 안전: 지상국이 보낼 수 있는 명령에 "발사"는 없다. 화약은 어떤 경우에도
// 무선으로 터뜨릴 수 없고, 기체 자신의 상태머신만 터뜨린다. 무선으로
// 가능한 것은 ARM / DISARM / PING 뿐이며, 비행 중에는 ARM/DISARM 도
// 무시된다.

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

constexpr uint8_t kSync0 = 0xA5;
constexpr uint8_t kSync1 = 0x5A;
constexpr uint8_t kVersion = 1;
constexpr size_t kHeaderBytes = 6;
constexpr size_t kCrcBytes = 2;
constexpr size_t kMaxPayloadBytes = 32;
constexpr size_t kMaxFrameBytes = kHeaderBytes + kMaxPayloadBytes + kCrcBytes;

enum class NodeId : uint8_t {
  Unknown = 0,
  Stage1 = 1,
  Stage2 = 2,
  Ground = 9,
};

enum class FrameType : uint8_t {
  Telemetry = 1,
  Command = 2,
  Ack = 3,
};

enum class RemoteCommand : uint8_t {
  None = 0,
  Arm = 1,
  Disarm = 2,
  Ping = 3,
};

// 상태 비트. 지상국에서 한눈에 보려고 묶어 둔다.
namespace StatusFlag {
constexpr uint8_t kSeparated = 1 << 0;
constexpr uint8_t kHeaterArmed = 1 << 1;
constexpr uint8_t kImuValid = 1 << 2;
constexpr uint8_t kBarometerValid = 1 << 3;
constexpr uint8_t kArmed = 1 << 4;
constexpr uint8_t kStageLinkPresentAtBoot = 1 << 5;
}  // namespace StatusFlag

// 24 바이트. 전체 프레임 32 바이트.
constexpr size_t kTelemetryPayloadBytes = 24;
struct TelemetryPayload {
  uint32_t timestampMs = 0;
  uint8_t state = 0;
  uint8_t reason = 0;
  uint8_t flags = 0;
  uint8_t reserved = 0;
  float altitudeM = 0.0f;
  int16_t verticalVelocityCmps = 0;  // m/s * 100
  int16_t accelerationMilliG = 0;    // g * 100
  int16_t maxAltitudeDm = 0;         // m * 10
  uint16_t batteryMilliV = 0;
  int16_t tiltDeciDeg = 0;  // deg * 10
  uint16_t droppedFrames = 0;
};

// 4 바이트. 전체 프레임 12 바이트.
constexpr size_t kCommandPayloadBytes = 4;
struct CommandPayload {
  uint8_t command = 0;        // RemoteCommand
  uint8_t target = 0;         // NodeId, 0xFF = 모두
  uint16_t nonce = 0;         // 중복 수신 무시용
};
constexpr uint8_t kTargetAll = 0xFF;

uint16_t crc16Ccitt(const uint8_t* data, size_t length);

// 반환값은 out 에 쓴 바이트 수. 버퍼가 작으면 0.
size_t encodeTelemetry(NodeId source, const TelemetryPayload& payload,
                       uint8_t* out, size_t capacity);
size_t encodeCommand(NodeId source, const CommandPayload& payload,
                     uint8_t* out, size_t capacity);

struct ParsedFrame {
  NodeId source = NodeId::Unknown;
  FrameType type = FrameType::Telemetry;
  uint8_t payload[kMaxPayloadBytes] = {};
  uint8_t payloadLen = 0;
};

bool decodeTelemetry(const ParsedFrame& frame, TelemetryPayload& out);
bool decodeCommand(const ParsedFrame& frame, CommandPayload& out);

// 바이트 스트림에서 프레임을 뽑아낸다. 수신기가 중간에 붙어도 sync 를
// 찾아 재동기한다. 한 바이트씩 넣고, 프레임이 완성되면 true.
class FrameReader {
 public:
  bool push(uint8_t byte, ParsedFrame& out);
  void reset() { count_ = 0; }
  uint32_t crcErrors() const { return crcErrors_; }

 private:
  uint8_t buffer_[kMaxFrameBytes] = {};
  size_t count_ = 0;
  uint32_t crcErrors_ = 0;
};

}  // namespace telemetry
