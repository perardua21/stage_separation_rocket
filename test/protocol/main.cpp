#include <cassert>
#include <cstdio>
#include <cstring>

#include "telemetry_protocol.h"

using namespace telemetry;

namespace {

TelemetryPayload samplePayload() {
  TelemetryPayload p;
  p.timestampMs = 123456;
  p.state = 11;
  p.reason = 16;
  p.flags = StatusFlag::kArmed | StatusFlag::kSeparated |
            StatusFlag::kBarometerValid;
  p.altitudeM = 181.25f;
  p.verticalVelocityCmps = -1234;
  p.accelerationMilliG = 97;
  p.maxAltitudeDm = 1812;
  p.batteryMilliV = 7946;
  p.tiltDeciDeg = -35;
  p.droppedFrames = 7;
  return p;
}

void testFrameSizes() {
  uint8_t buf[kMaxFrameBytes];
  assert(encodeTelemetry(NodeId::Stage1, samplePayload(), buf, sizeof(buf)) ==
         32);
  CommandPayload c;
  assert(encodeCommand(NodeId::Ground, c, buf, sizeof(buf)) == 12);
  // 버퍼가 작으면 아무것도 쓰지 않는다.
  assert(encodeTelemetry(NodeId::Stage1, samplePayload(), buf, 10) == 0);
}

void testTelemetryRoundTrip() {
  const TelemetryPayload sent = samplePayload();
  uint8_t buf[kMaxFrameBytes];
  const size_t n = encodeTelemetry(NodeId::Stage2, sent, buf, sizeof(buf));
  assert(n > 0);

  FrameReader reader;
  ParsedFrame frame;
  bool done = false;
  for (size_t i = 0; i < n; ++i) done = reader.push(buf[i], frame);
  assert(done);
  assert(frame.source == NodeId::Stage2);
  assert(frame.type == FrameType::Telemetry);

  TelemetryPayload got;
  assert(decodeTelemetry(frame, got));
  assert(got.timestampMs == sent.timestampMs);
  assert(got.state == sent.state);
  assert(got.reason == sent.reason);
  assert(got.flags == sent.flags);
  assert(got.altitudeM == sent.altitudeM);
  assert(got.verticalVelocityCmps == sent.verticalVelocityCmps);
  assert(got.accelerationMilliG == sent.accelerationMilliG);
  assert(got.maxAltitudeDm == sent.maxAltitudeDm);
  assert(got.batteryMilliV == sent.batteryMilliV);
  assert(got.tiltDeciDeg == sent.tiltDeciDeg);
  assert(got.droppedFrames == sent.droppedFrames);
}

void testCommandRoundTrip() {
  CommandPayload sent;
  sent.command = static_cast<uint8_t>(RemoteCommand::Arm);
  sent.target = kTargetAll;
  sent.nonce = 0xBEEF;

  uint8_t buf[kMaxFrameBytes];
  const size_t n = encodeCommand(NodeId::Ground, sent, buf, sizeof(buf));
  FrameReader reader;
  ParsedFrame frame;
  bool done = false;
  for (size_t i = 0; i < n; ++i) done = reader.push(buf[i], frame);
  assert(done);
  assert(frame.source == NodeId::Ground);

  CommandPayload got;
  assert(decodeCommand(frame, got));
  assert(got.command == sent.command);
  assert(got.target == sent.target);
  assert(got.nonce == sent.nonce);
  // 타입이 다르면 디코드를 거부한다.
  TelemetryPayload wrong;
  assert(!decodeTelemetry(frame, wrong));
}

// 비트가 하나 깨지면 CRC 가 걸러낸다.
void testCrcRejectsCorruption() {
  uint8_t buf[kMaxFrameBytes];
  const size_t n = encodeTelemetry(NodeId::Stage1, samplePayload(), buf,
                                   sizeof(buf));
  buf[10] ^= 0x01;

  FrameReader reader;
  ParsedFrame frame;
  bool done = false;
  for (size_t i = 0; i < n; ++i) done = reader.push(buf[i], frame) || done;
  assert(!done);
  assert(reader.crcErrors() == 1);
}

// 중간부터 붙어도 sync 를 찾아 재동기한다. 앞에 잡음이 붙은 경우.
void testResyncAfterGarbage() {
  uint8_t frameBytes[kMaxFrameBytes];
  const size_t n = encodeTelemetry(NodeId::Stage1, samplePayload(),
                                   frameBytes, sizeof(frameBytes));

  const uint8_t garbage[] = {0x00, 0xFF, 0xA5, 0xA5, 0x12, 0xA5, 0x00};
  FrameReader reader;
  ParsedFrame frame;
  bool done = false;
  for (size_t i = 0; i < sizeof(garbage); ++i) {
    done = reader.push(garbage[i], frame) || done;
  }
  assert(!done);
  for (size_t i = 0; i < n; ++i) done = reader.push(frameBytes[i], frame);
  assert(done && "잡음 뒤 프레임을 못 잡음");

  TelemetryPayload got;
  assert(decodeTelemetry(frame, got));
  assert(got.timestampMs == 123456);
}

// 프레임 두 개가 연달아 와도 둘 다 잡는다.
void testBackToBackFrames() {
  uint8_t a[kMaxFrameBytes];
  uint8_t b[kMaxFrameBytes];
  TelemetryPayload p1 = samplePayload();
  TelemetryPayload p2 = samplePayload();
  p2.timestampMs = 999;
  const size_t na = encodeTelemetry(NodeId::Stage1, p1, a, sizeof(a));
  const size_t nb = encodeTelemetry(NodeId::Stage2, p2, b, sizeof(b));

  FrameReader reader;
  ParsedFrame frame;
  int received = 0;
  uint32_t stamps[2] = {0, 0};
  for (size_t i = 0; i < na; ++i) {
    if (reader.push(a[i], frame)) {
      TelemetryPayload got;
      assert(decodeTelemetry(frame, got));
      stamps[received++] = got.timestampMs;
    }
  }
  for (size_t i = 0; i < nb; ++i) {
    if (reader.push(b[i], frame)) {
      TelemetryPayload got;
      assert(decodeTelemetry(frame, got));
      stamps[received++] = got.timestampMs;
    }
  }
  assert(received == 2);
  assert(stamps[0] == 123456 && stamps[1] == 999);
}

// 길이 필드가 말이 안 되면 버린다.
void testRejectsBadLength() {
  FrameReader reader;
  ParsedFrame frame;
  const uint8_t header[] = {kSync0, kSync1, kVersion, 1, 1, 0xFF};
  bool done = false;
  for (size_t i = 0; i < sizeof(header); ++i) {
    done = reader.push(header[i], frame) || done;
  }
  assert(!done);
}

}  // namespace

int main() {
  testFrameSizes();
  testTelemetryRoundTrip();
  testCommandRoundTrip();
  testCrcRejectsCorruption();
  testResyncAfterGarbage();
  testBackToBackFrames();
  testRejectsBadLength();
  std::printf("protocol tests passed\n");
  return 0;
}
