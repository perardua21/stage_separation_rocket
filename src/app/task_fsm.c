/**
 * @file task_fsm.c
 * @brief 상태 전이 전담. 구조만 잡은 뼈대다.
 *
 * 이 태스크만 상태를 바꾼다. 이 태스크는 핀을 직접 만지지 않는다.
 */
#include "rtos_topology.h"
#include "flight_config.h"
#include "fsm_core.h"
#include "state_snapshot.h"
#include "esp_timer.h"

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

void task_fsm(void *arg)
{
    role_t role = (role_t)(intptr_t)arg;

    fsm_ctx_t ctx;
    fsm_init(&ctx, role);

    nav_state_t nav = {0};
    const TickType_t period = pdMS_TO_TICKS(CFG_FSM_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        /* 1. 최신 추정값. 길이 1 덮어쓰기 큐라 없으면 직전 값을 그대로 쓴다. */
        xQueueReceive(g_q.nav, &nav, 0);

        /* 2. 쌓인 이벤트를 모두 소비한다. 한 주기에 여러 개가 올 수 있다. */
        flight_event_t ev;
        while (xQueueReceive(g_q.event, &ev, 0) == pdTRUE) {
            fsm_step(&ctx, &nav, &ev, now_ms());
        }

        /* 3. 이벤트가 없어도 주기 판정은 돈다 (타임아웃, 임계값 판정). */
        fsm_step(&ctx, &nav, NULL, now_ms());

        /* 4. 바뀐 상태를 공개한다. 여기서만 publish 한다. */
        snapshot_publish(&ctx.snap);

        vTaskDelayUntil(&last, period);
    }
}
