/**
 * @file fsm_core.h
 * @brief 역할 공통 FSM 엔진. 전이표는 역할별 파일이 제공한다.
 *
 * 엔진이 하는 일
 *   - 매 주기 nav/event를 모아 현재 상태의 tick 함수를 부른다
 *   - 전이가 나면 exit → enter를 부르고 스냅샷을 공개한다
 *   - 상태 전이와 판정 근거를 로그 큐에 넣는다
 *
 * 엔진이 하지 않는 일
 *   - 열선을 직접 켜지 않는다. act 큐로 요청만 한다
 */
#pragma once
#include "flight_types.h"

typedef struct fsm_ctx fsm_ctx_t;

/* 한 상태의 동작. 다음 상태를 돌려주면 전이, FS_COUNT면 유지. */
typedef flight_state_t (*state_tick_fn)(fsm_ctx_t *ctx, const nav_state_t *nav, uint32_t now_ms);
typedef void (*state_enter_fn)(fsm_ctx_t *ctx, uint32_t now_ms);
typedef void (*state_exit_fn)(fsm_ctx_t *ctx, uint32_t now_ms);

typedef struct {
    flight_state_t  id;
    const char     *name;
    state_enter_fn  on_enter;
    state_tick_fn   on_tick;
    state_exit_fn   on_exit;
} state_def_t;

/* 역할별 전이표 */
typedef struct {
    role_t role;
    const state_def_t *states;   /* FS_COUNT 길이, 안 쓰는 칸은 NULL */
    flight_state_t initial;
} fsm_table_t;

const fsm_table_t *fsm_table_booster(void);
const fsm_table_t *fsm_table_sustainer(void);

/* FSM이 들고 다니는 상태. 판정기 카운터도 여기 모은다. */
struct fsm_ctx {
    const fsm_table_t *table;
    state_snapshot_t   snap;
    recovery_cause_t   pending_cause;
    uint32_t           state_entered_ms;
    uint8_t            sep_retry;
    bool               heater_armed;
};

void fsm_init(fsm_ctx_t *ctx, role_t role);
void fsm_step(fsm_ctx_t *ctx, const nav_state_t *nav, const flight_event_t *ev, uint32_t now_ms);

/* 상태 안에서 쓰는 보조 함수 */
void fsm_goto_recovery(fsm_ctx_t *ctx, recovery_cause_t cause, uint32_t now_ms);
void fsm_request(fsm_ctx_t *ctx, act_cmd_id_t id, uint32_t arg);
uint32_t fsm_elapsed(const fsm_ctx_t *ctx, uint32_t now_ms);
