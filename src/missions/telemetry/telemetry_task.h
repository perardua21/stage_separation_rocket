#pragma once

#include <HardwareSerial.h>

#include "runtime_channels.h"
#include "telemetry_protocol.h"

// E22 로 텔레메트리를 내보내고, 지상국 명령을 받아 명령 큐에 넣는다.
//
// 송신: 고정 길이 바이너리 프레임 (telemetry_protocol.h). 1단과 2단이
//       같은 주파수를 쓰므로 송신 시각을 엇갈리게 한다.
// 수신: ARM / DISARM / PING 만. 화약을 터뜨리는 무선 명령은 프로토콜에
//       아예 없다. ARM/DISARM 도 지상 상태(Safe/Armed)에서만 받는다.
class TelemetryTask {
 public:
  TelemetryTask(VehicleRole role, RuntimeChannels& channels,
                RuntimeStats& stats)
      : channels_(channels), stats_(stats), radio_(1), role_(role) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  void sendTelemetry(const FlightSnapshot& frame);
  void pollRadio();
  void handleCommand(const telemetry::CommandPayload& payload);

  RuntimeChannels& channels_;
  RuntimeStats& stats_;
  HardwareSerial radio_;
  VehicleRole role_;
  telemetry::FrameReader reader_;
  uint16_t lastNonce_ = 0;
  bool haveNonce_ = false;
};
