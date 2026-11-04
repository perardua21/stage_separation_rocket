/**
 * @file eject_seq.h
 * @brief 열선 시퀀스. task_actuator만 실행한다.
 *
 * 화약과 다른 점
 *   - 끊어지는 데 수 초가 걸린다(설정 CFG_HEAT_PULSE_MS, 기본 3 s)
 *   - 전류가 크다. 15 A 퓨즈라 두 채널을 동시에 켜지 않는다
 *   - 배터리 전압이 떨어지면 시간이 늘어난다. 전압을 같이 기록한다
 *
 * 시퀀스 (FIRE_DEPLOY 기준)
 *   ARM ON → 100 ms → CH_A ON → 3 s → CH_A OFF → 1 s
 *          → CH_B ON → 3 s → CH_B OFF → ARM OFF → EV_HEATER_SEQ_DONE
 */
#pragma once
#include "flight_types.h"

typedef enum {
    HEAT_SEQ_NONE = 0,
    HEAT_SEQ_SEPARATION,   /* [B] 단 분리 열선 */
    HEAT_SEQ_DEPLOY,       /* 낙하산 사출 열선 (주 + 백업) */
} heater_seq_id_t;

/* 채널 배정. 역할마다 의미가 다르므로 표로 둔다.
 * TODO: 확정 필요(docs/04-open-items.md 3번).
 *   부스터   CH1 = 단 분리, CH2 = 낙하산 사출
 *   서스테이너 CH1 = ?,      CH2 = 낙하산 사출
 * 서스테이너가 2단 점화를 직접 한다면 채널이 하나 더 있어야 한다. */
typedef struct {
    int8_t ch_separation;  /* -1이면 그 역할에는 없음 */
    int8_t ch_deploy_main;
    int8_t ch_deploy_backup;
} heater_map_t;

const heater_map_t *heater_map_for(role_t role);

/* 비블로킹 시퀀서. task_actuator가 매 틱 부른다. */
typedef struct {
    heater_seq_id_t id;
    uint8_t  step;
    uint32_t step_started_ms;
    bool     running;
} heater_seq_t;

void heater_seq_start(heater_seq_t *s, heater_seq_id_t id, uint32_t now_ms);
bool heater_seq_tick(heater_seq_t *s, uint32_t now_ms);   /* 끝나면 true */
void heater_seq_abort(heater_seq_t *s, uint32_t now_ms);  /* 전 채널 + ARM OFF */
