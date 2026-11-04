/**
 * @file flight_config.h
 * @brief 비행 판정 임계값과 타이머. 코드에 숫자를 직접 쓰지 않는다.
 *
 * 값마다 출처를 적는다. 출처가 "TODO"인 값은 비행 빌드 전에 반드시 확정한다.
 * 시뮬 값은 OpenRocket [G144-P; H411-0.5] 기준이며, 이 PCB를 올릴 기체가
 * 같은 기체인지 확인이 필요하다(docs/04-open-items.md 1번).
 */
#pragma once
#include <stdint.h>

/* ── 센서 주기 ─────────────────────────────────────────── */
#define CFG_IMU_PERIOD_MS          5     /* 200 Hz. BNO085 SPI */
#define CFG_BARO_PERIOD_MS        20     /* 50 Hz. BMP390 */
#define CFG_FSM_PERIOD_MS          5
#define CFG_LOG_FAST_PERIOD_MS    10     /* 비행 중 100 Hz */
#define CFG_LOG_SLOW_PERIOD_MS    50     /* 하강 중 20 Hz */
#define CFG_TLM_PERIOD_MS        200     /* 5 Hz */

/* ── 공통 판정 ─────────────────────────────────────────── */
#define CFG_LAUNCH_ACC_G         2.0f   /* 출처: 작년 로그 + 팀 결정 */
#define CFG_LAUNCH_CONFIRM_MS     50    /* 연속 50 ms. 샘플 수가 아니라 시간으로 */
#define CFG_BURNOUT_ACC_G        2.0f   /* 이하. 출처: 상태머신 설계서 */
#define CFG_BURNOUT_CONFIRM_MS    50
#define CFG_APOGEE_DROP_M        3.0f   /* h_max 대비 하강량 */
#define CFG_APOGEE_CONFIRM_N      5     /* 연속 기압 샘플 (20 ms x 5 = 100 ms) */
#define CFG_BARO_LOCKOUT_MS      500    /* 분리·점화·사출 직후 기압 판정 중단 */
#define CFG_APOGEE_HOLD_MAX_MS   200    /* FS_APOGEE 최대 체류 */
#define CFG_LANDING_WINDOW_MS   5000
#define CFG_LANDING_RANGE_M      2.0f
#define CFG_DESCENT_WARN_MPS    15.0f

/* ── 분리 조건 ─────────────────────────────────────────────
 * 팀 결정(2026-11-04): 분리에 절대고도 조건을 추가한다.
 * 시간 조건만으로는 발사 실패·조기 연소종료 때 지면 가까이에서
 * 분리될 수 있기 때문.
 *
 * ⚠ 시뮬상 분리 고도는 약 65 m다. 100 m를 그대로 쓰면 시뮬 기준으로는
 *   분리가 약 0.6 s 늦어지고(100 m 도달 ≈ T0+2.1 s), 2단 점화도 같이
 *   밀린다. 기체가 바뀐 것인지, 100 m가 잠정값인지 확인할 것. */
#define CFG_SEP_MIN_ALT_M       100.0f  /* 팀 결정, 잠정 */
#define CFG_SEP_CMD_FROM_T0_MS   1500   /* TODO: 시뮬 분리 1.5 s */
#define CFG_SEP_CONFIRM_TO_MS     500   /* 명령 후 확인 대기 */
#define CFG_SEP_RETRY_COUNT         1
#define CFG_SEP_TIMEOUT_FROM_T0_MS 2500 /* 이후 RECOVERY(NO_SEP) */

/* ── 역할별: 부스터 ────────────────────────────────────── */
#define CFG_B_MIN_BURN_MS         500   /* TODO: 1단 모터 데이터시트 */
#define CFG_B_BURN_TIMEOUT_MS    1300   /* TODO: 연소시간 + 0.3 s */
#define CFG_B_APOGEE_MIN_FROM_T0_MS  3500  /* TODO: 시뮬 정점 5.2 s */
#define CFG_B_APOGEE_TIMEOUT_MS      6200  /* TODO: 시뮬 정점 + 1.0 s */
#define CFG_B_GROUND_TIMEOUT_MS     40000  /* TODO: 시뮬 하강 26.6 s x 1.5 */

/* ── 역할별: 서스테이너 ────────────────────────────────── */
#define CFG_S_IGN_WINDOW_MIN_MS    500  /* T_sep 이후 최소 대기(부스터와 거리 확보) */
#define CFG_S_IGN_WINDOW_MAX_MS   1500  /* 이 안에 못 쏘면 포기 */
#define CFG_S_IGN_MAX_TILT_DEG    20.0f /* 팀 결정. TODO: 풍속별 시뮬 */
#define CFG_S_IGN_MIN_VEL_MPS     30.0f /* 가속도 적분 속도. TODO: 시뮬 57 m/s */
#define CFG_S_IGN_MIN_ALT_M      100.0f /* 분리 조건과 같은 값 */
#define CFG_S_IGN_CONFIRM_ACC_G    3.0f /* 연소종료 2 G와 겹치지 않게 */
#define CFG_S_IGN_CONFIRM_TO_MS   1000  /* TODO: 점화기 지연 지상시험 */
#define CFG_S_MIN_BURN_MS         1000
#define CFG_S_BURN_TIMEOUT_MS     2200  /* TODO: 2단 연소 1.85 s + 0.3 s */
#define CFG_S_APOGEE_MIN_FROM_IGN_MS  6000
#define CFG_S_APOGEE_TIMEOUT_MS      12000 /* T_ign 기준 */
#define CFG_S_GROUND_TIMEOUT_MS     140000

/* ── 열선 출력 ─────────────────────────────────────────────
 * 화약(e-match)이 아니라 열선이다. 끊어지는 데 걸리는 시간이 훨씬 길고
 * 배터리 전압·열선 길이에 민감하다. 아래 값은 전부 지상 절단시험으로
 * 확정해야 한다. 15 A 퓨즈라 두 채널 동시 구동은 금지. 순차만. */
#define CFG_HEAT_ARM_SETTLE_MS     100  /* ARM 후 전원 안정 대기 */
#define CFG_HEAT_PULSE_MS         3000  /* TODO: 지상 절단시험. 화약 1 s와 다름 */
#define CFG_HEAT_GAP_MS           1000  /* 채널 사이 간격 */
#define CFG_HEAT_MAX_ACTIVE          1  /* 동시에 켤 수 있는 채널 수 */

/* ── 전원 ──────────────────────────────────────────────── */
#define CFG_BATT_CELLS              3
#define CFG_BATT_WARN_MV        10800   /* 3.6 V/셀 */
#define CFG_BATT_CRIT_MV        10200   /* 3.4 V/셀 */

/* ── 리셋 복구 ─────────────────────────────────────────── */
#define CFG_RESET_RECOVER_ALT_M  20.0f  /* 재부팅 시 이 고도 이상이면 RECOVERY */
