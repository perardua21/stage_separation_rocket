# 태스크 루프

여기 파일들은 "배선"만 한다. 판정 로직은 `src/flight/`, 하드웨어 접근은
`src/bsp/`에 있다. 태스크 파일에 임계값이나 판정식을 쓰지 않는다.

- `task_sensor.c` — BNO085 / BMP390을 읽어 `q_imu`, `q_baro`에 넣는다 (Core 0)
- `task_estimator.c` — 고도 필터, 수직속도, 가속도 적분 속도 → `q_nav` (Core 0)
- `task_fsm.c` — 상태 전이. 상태를 바꾸는 유일한 곳 (Core 1)
- `task_actuator.c` — 열선 시퀀스. 핀을 켜는 유일한 곳 (Core 1)
- `task_logger.c` — microSD 기록 (Core 1)
- `task_telemetry.c` — E22 송수신 (Core 1)
- `task_health.c` — 센서·전원 감시, 이벤트 발행 (Core 1)

작성 순서는 sensor → estimator → fsm → actuator → logger → telemetry → health를
권한다. 앞의 둘이 없으면 FSM을 리플레이로 검증할 수 없기 때문이다.
