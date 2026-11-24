#pragma once

#include "runtime_channels.h"

// 지상국이 없으므로 부저가 유일한 상태 표시 수단이다.
//
//   부팅     1단은 길게 한 번, 2단은 짧게 두 번 (역할 확인용)
//   SAFE     무음
//   ARMED    2 초마다 짧게 한 번 (무장 상태를 멀리서도 알 수 있게)
//   비행 중  무음
//   GROUND   1 초마다 길게 (착지 후 기체 찾기용 비컨)
//   FAULT    빠르게 세 번 반복
class BuzzerTask {
 public:
  explicit BuzzerTask(RuntimeStats& stats) : stats_(stats) {}

  // 태스크 시작 전에 setup() 에서 직접 호출한다.
  static void playRoleCue(VehicleRole role);

  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeStats& stats_;
};
