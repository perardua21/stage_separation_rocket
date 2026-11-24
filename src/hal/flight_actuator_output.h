#pragma once

#include "flight_types.h"
#include "hal/actuator_output.h"

// 실제 열선 출력을 구동한다. 비행 빌드에서만 링크된다.
//
// 하드웨어 구조 (PCB 넷리스트에서 확인)
//   HEAT_ARM (GPIO48) -> R46 2.2k -> Q12 MMBT3904(NPN) -> R54 4.7k
//                     -> Q15 SiRA99DP(P-ch) 게이트 -> HEAT_POWER
//     R47 100k 베이스 풀다운, R48 10k 게이트 풀업 -> 기본 OFF
//   HEAT_IN1 (GPIO3)  -> R42 100R -> Q10 AON7524(N-ch) 게이트, R44 10k 풀다운
//   HEAT_IN2 (GPIO7)  -> R43 100R -> Q11 AON7524(N-ch) 게이트, R45 10k 풀다운
//
// 둘 다 active HIGH 이고, 전류가 흐르려면 HEAT_ARM 과 채널이 동시에 켜져야
// 한다. 여기서도 소프트웨어로 같은 인터락을 한 번 더 건다.
class FlightActuatorOutput : public IActuatorOutput {
 public:
  void begin() override;
  void allOff() override;
  void apply(const ActuatorCommand& command) override;
  bool armed() const override { return armed_; }

 private:
  void writeChannel(ActuatorChannel channel, bool on);

  bool armed_ = false;
  ActuatorChannel active_ = ActuatorChannel::None;
};
