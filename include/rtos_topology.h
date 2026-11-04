/**
 * @file rtos_topology.h
 * @brief FreeRTOS 태스크·큐 토폴로지. 여기만 보면 동시성 구조가 보이도록.
 *
 * 핵심 규칙
 *  1. 상태는 큐로 보내지 않는다. 큐는 "이벤트"와 "샘플"만 나른다.
 *     현재 상태는 state_snapshot으로 공개한다(이유는 flight_types.h 주석).
 *  2. 열선을 건드리는 태스크는 task_actuator 하나뿐이다.
 *  3. 상태를 바꾸는 태스크는 task_fsm 하나뿐이다.
 *  4. 센서·추정은 Core 0, 판정·출력·통신·로깅은 Core 1.
 */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "flight_types.h"

/* ── 코어 배정 ─────────────────────────────────────────── */
#define CORE_SENSE   0   /* 센서 수집 + 추정 */
#define CORE_MISSION 1   /* FSM + 출력 + 통신 + 로깅 */

/* ── 우선순위 (높을수록 먼저) ──────────────────────────────
 * actuator가 fsm보다 높다. 사출 펄스 타이밍이 밀리면 안 되기 때문.
 * actuator는 명령을 받을 때만 잠깐 돌고 바로 블록된다. */
#define PRIO_ACTUATOR   23
#define PRIO_FSM        22
#define PRIO_SENSOR     20
#define PRIO_ESTIMATOR  18
#define PRIO_LOGGER     10
#define PRIO_TELEMETRY   8
#define PRIO_HEALTH      5

/* ── 스택 (바이트) ─────────────────────────────────────── */
#define STACK_SENSOR     4096
#define STACK_ESTIMATOR  6144
#define STACK_FSM        4096
#define STACK_ACTUATOR   2560
#define STACK_LOGGER     4096
#define STACK_TELEMETRY  4096
#define STACK_HEALTH     3072

/* ── 큐 깊이와 가득 찼을 때의 정책 ─────────────────────────
 *
 *  큐        깊이  생산자                소비자        가득 차면
 *  q_imu       8   task_sensor           estimator     가장 오래된 것 버림
 *  q_baro      8   task_sensor           estimator     가장 오래된 것 버림
 *  q_nav       1   estimator             task_fsm      덮어쓰기(최신만)
 *  q_event    16   health/tlm/actuator   task_fsm      10 ms 블록 후 드롭+카운트
 *  q_act       8   task_fsm              actuator      블록. 절대 드롭 금지
 *  q_log     256   모든 태스크           logger        드롭 + 드롭수 카운트
 *  q_tlm       4   fsm/estimator         telemetry     덮어쓰기
 *  q_uplink    4   telemetry             task_fsm      드롭
 *
 * q_nav는 길이 1 + xQueueOverwrite를 쓴다. 추정 결과는 최신값만 의미가 있고,
 * FSM이 한 주기 늦어도 다음 값으로 따라잡으면 되기 때문.
 * q_act는 유일하게 블로킹이다. 사출 명령을 잃으면 안 된다. */
#define QD_IMU      8
#define QD_BARO     8
#define QD_NAV      1
#define QD_EVENT   16
#define QD_ACT      8
#define QD_LOG    256
#define QD_TLM      4
#define QD_UPLINK   4

typedef struct {
    QueueHandle_t imu;
    QueueHandle_t baro;
    QueueHandle_t nav;
    QueueHandle_t event;
    QueueHandle_t act;
    QueueHandle_t log;
    QueueHandle_t tlm;
    QueueHandle_t uplink;
} queue_set_t;

extern queue_set_t g_q;

/* 큐 생성. 하나라도 실패하면 false. 큰 큐(q_log)는 PSRAM에 잡는다. */
bool rtos_queues_create(void);

/* 태스크 생성. xTaskCreatePinnedToCore로 코어 고정. */
bool rtos_tasks_start(role_t role);

/* 각 태스크 진입점 */
void task_sensor(void *arg);
void task_estimator(void *arg);
void task_fsm(void *arg);
void task_actuator(void *arg);
void task_logger(void *arg);
void task_telemetry(void *arg);
void task_health(void *arg);
