#include "app/flight_controller_app.h"

#include <Arduino.h>

#include "board_pins.h"
#include "flight_config.h"

namespace {
constexpr BaseType_t kStateCore = 1;
constexpr BaseType_t kIoCore = 0;
}  // namespace

bool FlightControllerApp::setup(uint32_t nowMs) {
  (void)nowMs;
  Serial.begin(BoardPins::kSerialBaud);
  delay(500);

  // 화약 출력 핀을 가장 먼저 확정한다.
  actuatorOutput_.begin();
  pinMode(BoardPins::kLedGreen, OUTPUT);
  pinMode(BoardPins::kLedRed, OUTPUT);
  pinMode(BoardPins::kBuzzer, OUTPUT);
  digitalWrite(BoardPins::kBuzzer, LOW);
  pinMode(BoardPins::kMain5vPresent, INPUT_PULLUP);
  pinMode(BoardPins::kStageId, INPUT_PULLUP);

#if defined(NURA_BENCH_SAFE)
  const char* outputs = "BLOCKED (bench)";
#else
  const char* outputs = "*** LIVE ***";
#endif

  // 역할은 빌드 타임에 고정된다(vehicle_role_config.h). J7 엇갈림 배선
  // 때문에 핀으로는 단 구분이 불가능하다. 보드를 바꿔 올리는 사고를 막기
  // 위해 배너를 크게 찍고 바로 아래에서 부저로도 알린다. 플래싱 전에
  // PCB 에 적힌 표시와 이 두 가지를 반드시 확인한다.
  Serial.println();
  Serial.println("========================================");
  Serial.printf("  ROLE    : %s\n", NURA_ROLE_NAME);
  Serial.printf("  OUTPUTS : %s\n", outputs);
  Serial.printf("  PYRO    : %s\n",
                kBuildRole == VehicleRole::Booster
                    ? "CH1 = separation + stage-1 chute (single charge)"
                    : "CH2 = stage-2 chute");
  Serial.println("========================================");
  Serial.println("buzzer: 1 long = STAGE-1, 2 short = STAGE-2");
  Serial.println("commands: arm | disarm | sep");

  BuzzerTask::playRoleCue(kBuildRole);

  if (!channels_.create()) failSafe("queue allocation");

  if (!imuTask_.start(kStateCore) ||
      !barometerTask_.start(kStateCore) ||
      !powerSenseTask_.start(kStateCore) ||
      !fsmTask_.start(kStateCore) ||
      !stageLinkTask_.start(kStateCore) ||
      !actuatorTask_.start(kStateCore) ||
      !flightLogTask_.start(kIoCore) ||
      !telemetryTask_.start(kIoCore) ||
      !commandTask_.start(kIoCore) ||
      !buzzerTask_.start(kIoCore)) {
    failSafe("task start");
  }
  return true;
}

void FlightControllerApp::loop(uint32_t nowMs) {
  (void)nowMs;
  vTaskDelay(portMAX_DELAY);
}

[[noreturn]] void FlightControllerApp::failSafe(const char* reason) {
  actuatorOutput_.allOff();
  Serial.printf("fatal initialization failure: %s\n", reason);
  abort();
}
