#pragma once

#include <stdint.h>

// 1단 화약 한 발이 단 분리와 1단 낙하산 사출을 동시에 수행하는 구성.
// 2단 모터는 없으며, 2단은 분리 후 관성 비행하다 자기 기압계로 정점을 잡아
// 낙하산만 사출한다.
namespace FlightConfig {

// ── 공통 판정 ────────────────────────────────────────────────
// 샘플 수는 IMU 100 Hz 기준이다. 5 샘플 = 50 ms.
constexpr float kLaunchThresholdG = 2.0f;
constexpr uint8_t kLaunchSamples = 5;
constexpr float kBurnoutThresholdG = 2.0f;
constexpr uint8_t kBurnoutSamples = 5;
constexpr float kApogeeDropM = 3.0f;
constexpr uint8_t kApogeeSamples = 5;
constexpr float kLandingRangeM = 2.0f;
constexpr uint32_t kLandingWindowMs = 5000;

// ── 연소 구간 ────────────────────────────────────────────────
// TODO: 1단 모터 데이터시트로 확정. 연소시간 + 0.3 s.
// 너무 짧게 잡혀 Coast로 일찍 들어가도 정점 판정에 별도 최소 시간
// 가드가 있어 조기 사출로는 이어지지 않는다.
constexpr uint32_t kBurnoutEarliestMs = 500;
constexpr uint32_t kBurnoutTimeoutMs = 1300;

// ── 정점 판정 ────────────────────────────────────────────────
// 기준 시각은 이륙(T0). 두 단이 붙은 채 정점까지 올라가므로 1단과 2단이
// 같은 값을 쓴다.
//
// 최소 비행시간 가드. 모터 점화 순간의 압력 교란으로 조기 사출되는 것만
// 막으면 되므로 짧게 잡는다. 길게 잡으면 스택 정점이 생각보다 이를 때
// 그만큼 사출이 늦어진다. 실제 정점 판정은 "최고점 대비 3 m 하강 + 수직속도
// 0 이하"라서 상승 중에는 성립하지 않는다.
constexpr uint32_t kApogeeEarliestFromLaunchMs = 2000;

// !! kApogeeTimeoutFromLaunchMs 는 확정값이 아니다 !!
// 기존 시뮬(181 m / 5.2 s)은 분리 후 1단 단독 비행 기준이라 이번 구성과
// 다르다. 분리하지 않는 스택 전체로 시뮬을 다시 돌려 정점 시각을 받은 뒤
// "정점 시각 + 1.5 s"로 교체할 것.
// 이 값은 기압계가 고장났을 때만 쓰이는 백업 경로다.
constexpr uint32_t kApogeeTimeoutFromLaunchMs = 9000;  // TODO: 재시뮬 필요

// "실제로 날아갔는가" 인터락. 모터 불발이나 CATO 로 기체가 발사대 근처에
// 주저앉으면 고도가 금방 음수가 되어 "최고점 대비 3 m 하강"이 성립하고,
// 지상에서 화약이 터진다. 최고 고도가 이 값을 넘긴 적이 없으면 기압 기반
// 정점 판정을 쓰지 않는다.
//
// 넘기지 못한 경우에도 kApogeeTimeoutFromLaunchMs 경로로는 사출된다.
// 즉 이 값을 올리면 "지상 폭발"은 막지만 저고도 비행에서 사출이 타임아웃까지
// 늦어진다. 30 m 는 발사대 안전과 사출 지연 사이의 타협값이다.
// 전에 말했던 100 m 는 2단 점화 조건이던 값이라 이 인터락에는 너무 높다
// (스택 정점이 100 m 부근이면 정상 비행에서도 판정이 안 걸린다).
constexpr float kMinApogeeAltitudeM = 30.0f;

// ── 사출 시퀀스 ──────────────────────────────────────────────
// HEAT_ARM(공통 전원)을 올리고 안정될 때까지 기다린 뒤 채널을 켠다.
// 두 채널은 절대 동시에 켜지 않는다(15 A 퓨즈).
//
// 전자팀 G 항목: 설계 한도는 채널당 12 A, 2 초 ON / 3 초 OFF.
// 따라서 펄스는 2000 ms 를 넘기지 않고, 채널 간 간격은 3000 ms 이상.
constexpr uint32_t kHeaterArmSettleMs = 100;
constexpr uint32_t kPyroPulseMs = 1000;
constexpr uint32_t kHeaterGapMs = 3000;
static_assert(kPyroPulseMs <= 2000, "G: channel ON limit is 2 s");
static_assert(kHeaterGapMs >= 3000, "G: channel OFF limit is 3 s");

// 2단은 1단의 분리 화약이 터지고 두 단이 떨어질 시간을 준 뒤 사출한다.
constexpr uint32_t kSustainerDeployDelayMs = 500;

// 1단 백업 화약을 실제로 장착했을 때만 true로 바꾼다. 화약이 없는데 true면
// 빈 채널에 전류만 흘린다.
constexpr bool kBoosterBackupChargeFitted = false;

// ── 착지 ─────────────────────────────────────────────────────
constexpr uint32_t kLandingTimeoutMs = 120000;

// ── 단 분리 감지 (J7, GPIO2) ─────────────────────────────────
// 두 보드의 J7 을 엇갈려 연결한다(1단 1번 ↔ 2단 2번, 1단 2번 ↔ 2단 1번).
// 붙어 있는 동안 두 보드 모두 LOW, 끊기면 두 보드 모두 HIGH.
// 두 보드가 같은 코드로 분리를 감지한다. 단 구분에는 쓸 수 없다
// (vehicle_role_config.h 참조).
//
// 부팅 때 이 시간 동안 LOW 를 확인해야 감지기가 활성화된다. 점퍼를 꽂지
// 않은 벤치 보드는 처음부터 HIGH 라서, 확인 없이 돌리면 부팅 직후
// "분리됨"으로 래치된다.
constexpr uint32_t kStageLinkBootCheckMs = 300;

// 분리는 영구적인 단선이므로 디바운스를 길게 잡아도 잃는 것이 없다. 짧게
// 잡으면 비행 중 진동으로 커넥터가 순간 떨어질 때 가짜 분리로 래치된다.
constexpr uint32_t kSeparationDebounceMs = 100;

// ── 무선 ─────────────────────────────────────────────────────
// E22 투명 전송 모드. 세 노드가 같은 주파수를 공유하므로 1단과 2단이
// 송신 시각을 엇갈리게 쓴다.
//
// 공중 전송률 확인 필요. E22 기본값 2.4 kbps 라면 32 바이트 프레임 하나가
// 약 107 ms 를 쓴다. 두 노드가 각 2 Hz 면 초당 428 ms 를 쓰는 셈이라
// 여유가 거의 없다. 2.4 kbps 를 그대로 쓸 경우 kTelemetryPeriodMs 를
// 1000 으로 올리거나, 모듈을 9.6 kbps 이상으로 설정할 것.
constexpr uint32_t kTelemetryPeriodMs = 500;
constexpr uint32_t kStage1SlotOffsetMs = 0;
constexpr uint32_t kStage2SlotOffsetMs = 250;

// 무선 ARM/DISARM 은 지상 상태에서만 받는다. 비행 중에는 무시한다.
// 무선으로 화약을 터뜨리는 명령은 프로토콜에 존재하지 않는다.
constexpr bool kAcceptRemoteArm = true;

}  // namespace FlightConfig
