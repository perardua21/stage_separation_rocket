#include "telemetry_protocol.h"

#include <string.h>

namespace telemetry {
namespace {

size_t encode(NodeId source, FrameType type, const void* payload,
              size_t payloadLen, uint8_t* out, size_t capacity) {
  const size_t total = kHeaderBytes + payloadLen + kCrcBytes;
  if (payloadLen > kMaxPayloadBytes || capacity < total) return 0;
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = kVersion;
  out[3] = static_cast<uint8_t>(source);
  out[4] = static_cast<uint8_t>(type);
  out[5] = static_cast<uint8_t>(payloadLen);
  memcpy(out + kHeaderBytes, payload, payloadLen);
  const uint16_t crc = crc16Ccitt(out + 2, 4 + payloadLen);
  out[kHeaderBytes + payloadLen] = static_cast<uint8_t>(crc & 0xFF);
  out[kHeaderBytes + payloadLen + 1] = static_cast<uint8_t>(crc >> 8);
  return total;
}

}  // namespace

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t encodeTelemetry(NodeId source, const TelemetryPayload& payload,
                       uint8_t* out, size_t capacity) {
  // 구조체를 그대로 보내지 않고 바이트를 직접 깐다. 패딩과 정렬이
  // 컴파일러마다 달라 지상국이 다른 툴체인이어도 깨지지 않게 한다.
  uint8_t buf[kTelemetryPayloadBytes];
  memcpy(buf + 0, &payload.timestampMs, 4);
  buf[4] = payload.state;
  buf[5] = payload.reason;
  buf[6] = payload.flags;
  buf[7] = payload.reserved;
  memcpy(buf + 8, &payload.altitudeM, 4);
  memcpy(buf + 12, &payload.verticalVelocityCmps, 2);
  memcpy(buf + 14, &payload.accelerationMilliG, 2);
  memcpy(buf + 16, &payload.maxAltitudeDm, 2);
  memcpy(buf + 18, &payload.batteryMilliV, 2);
  memcpy(buf + 20, &payload.tiltDeciDeg, 2);
  memcpy(buf + 22, &payload.droppedFrames, 2);
  return encode(source, FrameType::Telemetry, buf, sizeof(buf), out, capacity);
}

size_t encodeCommand(NodeId source, const CommandPayload& payload,
                     uint8_t* out, size_t capacity) {
  uint8_t buf[kCommandPayloadBytes];
  buf[0] = payload.command;
  buf[1] = payload.target;
  memcpy(buf + 2, &payload.nonce, 2);
  return encode(source, FrameType::Command, buf, sizeof(buf), out, capacity);
}

bool decodeTelemetry(const ParsedFrame& frame, TelemetryPayload& out) {
  if (frame.type != FrameType::Telemetry ||
      frame.payloadLen != kTelemetryPayloadBytes) {
    return false;
  }
  const uint8_t* b = frame.payload;
  memcpy(&out.timestampMs, b + 0, 4);
  out.state = b[4];
  out.reason = b[5];
  out.flags = b[6];
  out.reserved = b[7];
  memcpy(&out.altitudeM, b + 8, 4);
  memcpy(&out.verticalVelocityCmps, b + 12, 2);
  memcpy(&out.accelerationMilliG, b + 14, 2);
  memcpy(&out.maxAltitudeDm, b + 16, 2);
  memcpy(&out.batteryMilliV, b + 18, 2);
  memcpy(&out.tiltDeciDeg, b + 20, 2);
  memcpy(&out.droppedFrames, b + 22, 2);
  return true;
}

bool decodeCommand(const ParsedFrame& frame, CommandPayload& out) {
  if (frame.type != FrameType::Command ||
      frame.payloadLen != kCommandPayloadBytes) {
    return false;
  }
  out.command = frame.payload[0];
  out.target = frame.payload[1];
  memcpy(&out.nonce, frame.payload + 2, 2);
  return true;
}

bool FrameReader::push(uint8_t byte, ParsedFrame& out) {
  // sync 를 못 맞춘 상태면 첫 두 바이트를 맞출 때까지 버린다.
  if (count_ == 0) {
    if (byte != kSync0) return false;
    buffer_[count_++] = byte;
    return false;
  }
  if (count_ == 1) {
    if (byte != kSync1) {
      // 0xA5 0xA5 ... 처럼 연달아 오는 경우를 흘리지 않는다.
      count_ = (byte == kSync0) ? 1 : 0;
      buffer_[0] = kSync0;
      return false;
    }
    buffer_[count_++] = byte;
    return false;
  }

  buffer_[count_++] = byte;

  if (count_ == kHeaderBytes) {
    const uint8_t version = buffer_[2];
    const uint8_t payloadLen = buffer_[5];
    if (version != kVersion || payloadLen > kMaxPayloadBytes) {
      reset();
    }
    return false;
  }
  if (count_ < kHeaderBytes) return false;

  const size_t payloadLen = buffer_[5];
  const size_t total = kHeaderBytes + payloadLen + kCrcBytes;
  if (count_ < total) return false;

  const uint16_t received = static_cast<uint16_t>(
      buffer_[total - 2] | (static_cast<uint16_t>(buffer_[total - 1]) << 8));
  const uint16_t computed = crc16Ccitt(buffer_ + 2, 4 + payloadLen);
  reset();
  if (received != computed) {
    ++crcErrors_;
    return false;
  }

  out.source = static_cast<NodeId>(buffer_[3]);
  out.type = static_cast<FrameType>(buffer_[4]);
  out.payloadLen = static_cast<uint8_t>(payloadLen);
  memcpy(out.payload, buffer_ + kHeaderBytes, payloadLen);
  return true;
}

}  // namespace telemetry
