#pragma once

#include "runtime_channels.h"

// J7(GPIO2) 단 분리 감지. 1단과 2단에서 똑같이 동작한다.
//
// 배선 (전자팀 A 항목): 두 보드의 J7 두 핀을 엇갈려 연결한다.
//   1단 J7 1번(+) ─── 2단 J7 2번(-)
//   1단 J7 2번(-) ─── 2단 J7 1번(+)
//
// 각 보드의 1번 핀은 STAGE_ID(GPIO2, 10k 로 3V3 풀업), 2번 핀은 GND 다.
// 그래서 붙어 있는 동안 두 보드의 GPIO2 가 모두 상대 보드의 GND 로 끌려
// LOW 가 되고, 분리되면 각자의 풀업으로 둘 다 HIGH 가 된다.
//
//   LOW  = 두 단이 붙어 있음
//   HIGH = 분리됨
//
// 1단에게는 "화약이 실제로 단을 떼어냈다"는 확인 신호이고(로그용),
// 2단에게는 기압계가 죽었을 때 쓰는 보조 정점 신호다.
//
// 부팅 때 LOW 를 확인하지 못하면(점퍼 미연결 상태의 벤치 보드) 감지기를
// 활성화하지 않는다. 그러지 않으면 부팅 직후 분리됨으로 래치된다.
class StageLinkTask {
 public:
  StageLinkTask(RuntimeChannels& channels, RuntimeStats& stats)
      : channels_(channels), stats_(stats) {}
  bool start(BaseType_t core);

 private:
  static void entry(void* context);
  void run();
  RuntimeChannels& channels_;
  RuntimeStats& stats_;
};
