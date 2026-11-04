/**
 * @file task_actuator.c
 * @brief 열선 출력 전담. 핀을 켜는 유일한 태스크.
 *
 * 우선순위가 FSM보다 높다. 펄스 길이가 밀리면 안 되기 때문이다.
 * 명령을 기다릴 때는 큐에서 블록되므로 CPU를 쓰지 않는다.
 */
#include "rtos_topology.h"
#include "flight_config.h"
#include "bsp_heater.h"
#include "bsp_indicator.h"
#include "eject_seq.h"
#include "esp_timer.h"

static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

void task_actuator(void *arg)
{
    (void)arg;
    heater_seq_t seq = {0};

    for (;;) {
        act_cmd_t cmd;
        /* 시퀀스가 도는 중에는 짧게, 쉬는 중에는 길게 기다린다. */
        TickType_t wait = seq.running ? pdMS_TO_TICKS(5) : portMAX_DELAY;

        if (xQueueReceive(g_q.act, &cmd, wait) == pdTRUE) {
            switch (cmd.id) {
            case ACT_HEATER_ARM:
                bsp_heater_arm(true);
                break;
            case ACT_HEATER_DISARM:
                heater_seq_abort(&seq, now_ms());
                bsp_heater_arm(false);
                break;
            case ACT_HEATER_FIRE_SEQ:
                /* 이미 돌고 있으면 무시한다. 두 번 쏘지 않는다. */
                if (!seq.running) {
                    heater_seq_start(&seq, (heater_seq_id_t)cmd.arg, now_ms());
                }
                break;
            case ACT_BUZZER_PATTERN:
                bsp_buzzer_pattern((buzzer_pattern_t)cmd.arg);
                break;
            default:
                break;
            }
        }

        /* 비블로킹 시퀀서. 끝나면 FSM에 알린다. */
        if (seq.running && heater_seq_tick(&seq, now_ms())) {
            flight_event_t done = { .id = EV_HEATER_SEQ_DONE, .t_ms = now_ms() };
            xQueueSend(g_q.event, &done, pdMS_TO_TICKS(10));
        }
    }
}
