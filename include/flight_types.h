/**
 * @file flight_types.h
 * @brief 전 모듈이 공유하는 타입. 구현 없음, 선언만.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── 단 역할 ───────────────────────────────────────────────
 * 빌드 플래그가 아니라 부팅 시 STAGE_ID 핀으로 정한다.
 * 보드 두 장에 같은 바이너리를 올릴 수 있다.
 * J7에 10k 풀업이 있으므로 점퍼를 꽂으면 LOW.
 * 점퍼를 꽂아야 2단이 되도록 둔다(명시적 동작 쪽이 더 안전). */
typedef enum {
    ROLE_UNKNOWN = 0,
    ROLE_BOOSTER,    /* STAGE_ID = HIGH (점퍼 없음) */
    ROLE_SUSTAINER,  /* STAGE_ID = LOW  (점퍼 꽂음) */
} role_t;

/* ── 비행 상태 ─────────────────────────────────────────────
 * 두 역할이 같은 enum을 쓴다. 텔레메트리 상태 필드가 4비트라 0~15까지만.
 * 단별 상태(3~7, 14)는 같은 코드를 쓰고, 어느 단인지는 역할로 구분한다.
 * 운용개념서 v0.5의 BOOSTER_*/SUSTAINER_* 표기와 1:1 대응. */
typedef enum {
    FS_INIT        = 0,   /* 초기화·셀프테스트·기준값 측정 */
    FS_SAFE        = 1,   /* 전원 ON, 열선 차단, 아밍 대기 */
    FS_ARMED       = 2,   /* 발사 감지 대기 */
    FS_BOOST       = 3,   /* 자체 추진 (B: 1단 연소, S: 2단 연소) */
    FS_COAST       = 4,   /* 비추진 상승, 정점 감지 */
    FS_APOGEE      = 5,   /* 정점 판정 확정·기록. 출력 아님. 체류 짧음 */
    FS_DEPLOY      = 6,   /* 사출 열선 시퀀스 실행 중 */
    FS_DESCENT     = 7,   /* 낙하산 하강, 착지 판정 */
    FS_GROUND      = 8,   /* 착지, 로그 마감, 비콘 */
    FS_FAULT       = 9,   /* 초기화 실패 전용. 비행 불가 */
    FS_SEPARATING  = 10,  /* [B] 분리 명령·확인 */
    FS_RIDE        = 11,  /* [S] 1단에 실려 상승 */
    FS_SEPARATED   = 12,  /* [S] 분리 후 점화 조건 검사 */
    FS_IGNITING    = 13,  /* [S] 점화 명령 후 추력 확인 */
    FS_RECOVERY    = 14,  /* 고장 복구: 점화 금지 + 정점 사출은 수행 */
    FS_RESERVED15  = 15,
    FS_COUNT
} flight_state_t;

const char *flight_state_name(flight_state_t s);

/* ── RECOVERY 진입 원인 ────────────────────────────────── */
typedef enum {
    RC_NONE = 0,
    RC_NO_SEP,        /* 분리 미확인 */
    RC_NO_IGN,        /* [S] 점화 미확인 */
    RC_TILT,          /* [S] 기울기 초과 */
    RC_LOW_ALT,       /* 고도 조건 미달 */
    RC_LOW_VEL,       /* [S] 속도 조건 미달 */
    RC_LATE,          /* 허용 창 초과 */
    RC_SENSOR,        /* 비행 중 치명 센서 고장 */
    RC_RESET,         /* 비행 중 리셋 후 복귀 */
} recovery_cause_t;

/* ── 센서 샘플 ─────────────────────────────────────────── */
typedef struct {
    uint32_t t_ms;
    float    acc_mps2[3];   /* 센서 좌표계 */
    float    gyr_dps[3];
    float    quat[4];       /* BNO085 융합 출력 w,x,y,z */
    float    tilt_deg;      /* 수직 대비 각도 */
    bool     quat_valid;
    bool     acc_saturated; /* TODO: BNO085 레인지 포화 플래그. 04-open-items 참고 */
} imu_sample_t;

typedef struct {
    uint32_t t_ms;
    float    pressure_pa;
    float    temp_c;
    bool     valid;
} baro_sample_t;

/* ── 추정 결과 (estimator → fsm) ───────────────────────── */
typedef struct {
    uint32_t t_ms;
    float    alt_agl_m;     /* 발사 지점 기준 상대고도, 필터 통과 */
    float    alt_max_m;     /* COAST 중 갱신한 최고값 */
    float    vz_mps;        /* 수직속도 (기압 미분) */
    float    vz_imu_mps;    /* 가속도 적분 속도 (기압 락아웃 중 사용) */
    float    acc_norm_g;    /* 가속도 크기 */
    float    tilt_deg;
    bool     baro_ok;
    bool     imu_ok;
} nav_state_t;

/* ── FSM 입력 이벤트 ───────────────────────────────────── */
typedef enum {
    EV_NONE = 0,
    EV_ARM_REQUEST,        /* 지상국 ARM 명령 */
    EV_DISARM_REQUEST,
    EV_SEP_DETECTED,       /* 분리 확인. TODO: 현재 보드에 입력 핀 없음 */
    EV_HEATER_SEQ_DONE,    /* 액추에이터가 시퀀스를 끝냄 */
    EV_SENSOR_FAULT,       /* 헬스 태스크가 올림 */
    EV_SENSOR_RECOVERED,
    EV_FORCE_DEPLOY,       /* 지상국 강제 사출 */
} flight_event_id_t;

typedef struct {
    flight_event_id_t id;
    uint32_t t_ms;
    uint32_t arg;
} flight_event_t;

/* ── 액추에이터 명령 (fsm → actuator) ──────────────────── */
typedef enum {
    ACT_NONE = 0,
    ACT_HEATER_ARM,        /* 공통 전원만 올림 */
    ACT_HEATER_DISARM,
    ACT_HEATER_FIRE_SEQ,   /* arg = heater_seq_id_t */
    ACT_BUZZER_PATTERN,    /* arg = buzzer_pattern_t */
    ACT_LED_PATTERN,
} act_cmd_id_t;

typedef struct {
    act_cmd_id_t id;
    uint32_t arg;
} act_cmd_t;

/* ── 상태 스냅샷 ───────────────────────────────────────────
 * FSM이 쓰고 나머지가 읽는다. 큐가 아니라 seqlock으로 공개한다.
 * 이유: 상태는 "흐름"이 아니라 "현재 값"이고, 읽는 쪽이 여럿이며,
 * 한 번 놓쳐도 다음 주기에 최신값을 읽으면 되기 때문. 큐로 뿌리면
 * 소비자마다 복사본이 생기고 느린 소비자가 큐를 막는다. */
typedef struct {
    uint32_t seq;          /* 짝수 = 안정, 홀수 = 쓰는 중 */
    role_t   role;
    flight_state_t state;
    recovery_cause_t cause;
    uint32_t t0_ms;        /* 이륙 */
    uint32_t t_sep_ms;     /* 분리 확인 */
    uint32_t t_ign_ms;     /* [S] 점화 확인 */
    uint32_t t_apogee_ms;  /* 정점 확정 */
    nav_state_t nav;
    uint16_t health_flags;
    bool     ignition_inhibited;
} state_snapshot_t;

/* ── 헬스 플래그 (비행 상태와 다른 축) ─────────────────────
 * 운용개념서 8장: 센서·통신·저장 이상은 상태를 바꾸지 않고 따로 기록한다.
 * 상태를 바꾸는 것은 "발사 이후의 치명 센서 고장"뿐이다. */
enum {
    HF_IMU_FAIL    = 1u << 0,
    HF_BARO_FAIL   = 1u << 1,
    HF_SD_FAIL     = 1u << 2,
    HF_RADIO_FAIL  = 1u << 3,
    HF_BATT_LOW    = 1u << 4,
    HF_5V_LOST     = 1u << 5,
    HF_HEATER_ARMED= 1u << 6,
    HF_LOG_DROP    = 1u << 7,
};
