#include <Arduino.h>

#include "app/flight_controller_app.h"

// 빌드 환경이 두 가지다.
//
//   pio run -e bench   (기본값) 열선 출력이 전부 막힌다. 보드에 화약이 달려
//                      있어도 안전하다. 상태머신·센서·로그 확인은 여기서.
//   pio run -e flight            열선에 실제로 전류가 흐른다.
//
// default_envs = bench 이므로 그냥 pio run 하면 항상 안전한 쪽이 나온다.
// flight 는 -e flight 를 직접 쳐야 나오는, 의도적인 선택이다.
//
// flight 를 올리기 전에 끝내야 하는 것
//   1) 화약을 빼고 열선 저항만 달아 CH1/CH2 발사 시험
//   2) ARM 없이 CH 만 켰을 때 HEAT_POWER 에 전압이 안 걸리는지 확인
//   3) 1단/2단 보드를 바꿔 꽂지 않았는지 부팅 부저로 확인
//      1단 = 길게 한 번, 2단 = 짧게 두 번
//
// 아래 가드는 둘 중 아무 환경도 아닌 빌드(예: bench 환경에서 실수로
// NURA_BENCH_SAFE 를 지운 경우)를 막는다.
#if !defined(NURA_BENCH_SAFE) && !defined(NURA_FLIGHT_OUTPUTS_VALIDATED)
#error "Build with -e bench (outputs blocked) or -e flight (live outputs). Neither flag is set."
#endif

#if defined(NURA_BENCH_SAFE) && defined(NURA_FLIGHT_OUTPUTS_VALIDATED)
#error "bench and flight flags are both set; pick one environment."
#endif

namespace {
FlightControllerApp app;
}

void setup() { app.setup(millis()); }

void loop() { app.loop(millis()); }
