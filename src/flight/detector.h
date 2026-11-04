/**
 * @file detector.h
 * @brief 판정기. 상태를 바꾸지 않고 "조건 충족 여부"와 근거만 돌려준다.
 *
 * 두 역할이 같은 판정기를 임계값만 바꿔서 쓴다.
 * 모든 판정은 "연속 N ms" 방식이다. 조건이 한 번이라도 깨지면 0으로 되돌린다.
 */
#pragma once
#include "flight_types.h"

/* 연속 조건 카운터 */
typedef struct {
    uint32_t first_true_ms;
    bool     active;
} hold_t;

void hold_reset(hold_t *h);
bool hold_update(hold_t *h, bool cond, uint32_t now_ms, uint32_t need_ms);

/* 발사 / 연소종료 */
typedef struct { hold_t h; } launch_det_t;
bool det_launch(launch_det_t *d, const nav_state_t *nav, uint32_t now_ms);
bool det_burnout(launch_det_t *d, const nav_state_t *nav, uint32_t now_ms, uint32_t min_burn_ms, uint32_t t_ref_ms);

/* 정점. 기압 하강이 주 판정이고 타이머가 백업이다.
 * 목표고도는 판정에 쓰지 않는다. 성능이 모자라면 영영 도달하지 못하고,
 * 넘치면 상승 중에 사출되기 때문. 로그 비교용으로만 남긴다. */
typedef struct {
    float    alt_max_m;
    uint8_t  drop_count;
    uint32_t lockout_until_ms;
} apogee_det_t;
void apogee_det_reset(apogee_det_t *d, float alt_now_m);
void apogee_det_lockout(apogee_det_t *d, uint32_t until_ms);
bool det_apogee(apogee_det_t *d, const nav_state_t *nav, uint32_t now_ms, uint32_t min_from_ref_ms, uint32_t t_ref_ms);

/* 착지 */
typedef struct {
    float    win_min_m, win_max_m;
    uint32_t win_start_ms;
    bool     active;
} landing_det_t;
void landing_det_reset(landing_det_t *d);
bool det_landing(landing_det_t *d, const nav_state_t *nav, uint32_t now_ms);

/* ── 분리 게이트 (부스터) ──────────────────────────────────
 * 팀 결정으로 절대고도 조건이 들어간다. 시간만으로는 조기 연소종료 때
 * 지면 가까이에서 분리될 수 있기 때문. */
typedef struct {
    bool t_ok;      /* T0 + CFG_SEP_CMD_FROM_T0_MS 경과 */
    bool alt_ok;    /* alt_agl >= CFG_SEP_MIN_ALT_M */
    bool vz_ok;     /* 아직 상승 중 */
} sep_gate_t;
bool gate_separation(sep_gate_t *g, const nav_state_t *nav, uint32_t now_ms, uint32_t t0_ms);

/* ── 점화 게이트 (서스테이너) ─────────────────────────────
 * 5개 조건을 모두 만족해야 점화한다. 조건별 판정값을 그대로 로그에 남겨
 * 비행 후에 왜 쐈는지 / 왜 안 쐈는지 알 수 있게 한다.
 * 속도는 기압이 아니라 가속도 적분을 쓴다. 분리 직후 0.5 s는 기압
 * 락아웃 중이라 기압 속도를 믿을 수 없기 때문. */
typedef struct {
    bool  win_min_ok;   /* T_sep + 0.5 s 이상 */
    bool  win_max_ok;   /* T_sep + 1.5 s 이내 */
    bool  tilt_ok;      /* < 20 deg */
    bool  vel_ok;       /* 적분 속도 > 30 m/s */
    bool  alt_ok;       /* >= 100 m */
    float tilt_deg, vel_mps, alt_m;
} ign_gate_t;
bool gate_ignition(ign_gate_t *g, const nav_state_t *nav, uint32_t now_ms, uint32_t t_sep_ms);
recovery_cause_t gate_ignition_failure_cause(const ign_gate_t *g);
