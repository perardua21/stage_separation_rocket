#pragma once

#include "flight_types.h"

// 역할은 빌드 타임에 고정된다.
//
// 왜 핀으로 안 읽는가: J7(GPIO2) 두 핀을 두 보드 사이에 엇갈려 연결하기로
// 결정됐다. 1단의 1번 핀이 2단의 2번 핀에, 1단의 2번 핀이 2단의 1번 핀에
// 붙는다. 그래서 연결되어 있는 동안 두 보드의 STAGE_ID 가 모두 상대 보드의
// GND 로 끌려 LOW 가 되고, 분리되면 각자의 10k 풀업으로 둘 다 HIGH 가 된다.
// 즉 J7 은 분리 감지에만 쓸 수 있고 단 구분에는 쓸 수 없다.
//
// 따라서 1단 펌웨어와 2단 펌웨어가 서로 다른 바이너리다. 보드를 바꿔
// 올리면 아무 경고 없이 잘못 동작한다. 그걸 막기 위해
//   - 역할을 지정하지 않으면 컴파일이 되지 않는다 (아래 #error)
//   - 부팅 때 시리얼 배너, 부저 패턴, LED 로 역할을 알린다
//   - 모든 텔레메트리 프레임에 송신 노드 ID 가 들어간다
// 플래싱 전에 PCB 에 적힌 표시와 부팅 부저를 반드시 확인한다.

#if defined(NURA_ROLE_BOOSTER) && defined(NURA_ROLE_SUSTAINER)
#error "Both NURA_ROLE_BOOSTER and NURA_ROLE_SUSTAINER are set; pick one."
#elif defined(NURA_ROLE_BOOSTER)
constexpr VehicleRole kBuildRole = VehicleRole::Booster;
#define NURA_ROLE_NAME "STAGE-1 BOOSTER"
#elif defined(NURA_ROLE_SUSTAINER)
constexpr VehicleRole kBuildRole = VehicleRole::Sustainer;
#define NURA_ROLE_NAME "STAGE-2 SUSTAINER"
#else
#error "Vehicle role is not set. Use one of: -e bench_s1, -e bench_s2, -e flight_s1, -e flight_s2."
#endif
