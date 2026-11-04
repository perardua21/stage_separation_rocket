/**
 * @file main.c
 * @brief 부팅 순서와 태스크 기동. 비행 로직은 여기에 두지 않는다.
 *
 * 부팅 순서가 안전에 직결된다. 열선 출력을 가장 먼저 LOW로 고정하고,
 * 그다음에 역할을 읽고, 센서를 올린 뒤 태스크를 띄운다.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "board_pins.h"
#include "flight_config.h"
#include "flight_types.h"
#include "rtos_topology.h"
#include "bsp_heater.h"
#include "bsp_indicator.h"
#include "bsp_power.h"
#include "state_snapshot.h"

static const char *TAG = "main";

/**
 * STAGE_ID 핀으로 역할을 읽는다.
 * 빌드 플래그가 아니라 핀으로 정하므로 보드 두 장에 같은 바이너리를 올린다.
 * 진동·접촉 불량으로 한 번 잘못 읽으면 2단이 1단처럼 동작할 수 있으므로
 * 20 ms 간격 5회를 읽어 전부 같을 때만 확정한다.
 */
static role_t read_stage_role(void)
{
    /* TODO: PIN_STAGE_ID 확정 후 구현.
     *   level = gpio_get_level(PIN_STAGE_ID);
     *   5회 연속 일치해야 확정, 아니면 ROLE_UNKNOWN
     *   HIGH(점퍼 없음) = ROLE_BOOSTER, LOW(점퍼 꽂음) = ROLE_SUSTAINER */
    return ROLE_UNKNOWN;
}

void app_main(void)
{
    /* 1. 열선부터 끈다. 다른 어떤 초기화보다 먼저.
     *    하드웨어에도 게이트 풀다운이 있지만 소프트웨어에서도 명시한다. */
    bsp_heater_init_safe();

    ESP_LOGI(TAG, "stage_separation_rocket boot");

    /* 2. 표시장치. 이후 단계가 실패해도 소리로 알릴 수 있게 먼저 올린다. */
    bsp_indicator_init();

    /* 3. 역할 확정. 실패하면 비행 금지. */
    role_t role = read_stage_role();
    if (role == ROLE_UNKNOWN) {
        ESP_LOGE(TAG, "STAGE_ID unreadable -> FAULT");
        bsp_indicator_panic(PANIC_STAGE_ID);   /* 부저 패턴 반복, 여기서 정지 */
        return;
    }
    ESP_LOGI(TAG, "role = %s", role == ROLE_BOOSTER ? "BOOSTER" : "SUSTAINER");

    /* 4. 전원 확인. 배터리가 낮으면 아밍을 거부해야 하므로 여기서 읽어둔다. */
    bsp_power_init();

    /* 5. 상태 스냅샷 초기화 (FS_INIT) */
    snapshot_init(role);

    /* 6. 큐 → 태스크 순서. 큐가 없으면 태스크가 바로 죽는다. */
    if (!rtos_queues_create()) {
        ESP_LOGE(TAG, "queue alloc failed");
        bsp_indicator_panic(PANIC_ALLOC);
        return;
    }
    if (!rtos_tasks_start(role)) {
        ESP_LOGE(TAG, "task create failed");
        bsp_indicator_panic(PANIC_TASK);
        return;
    }

    /* 이후 app_main은 할 일이 없다. 센서 초기화·셀프테스트·기준기압 측정은
     * FS_INIT 상태에서 task_fsm이 진행한다. 부팅 함수에서 블로킹하면
     * 워치독이 걸리고, 실패 시 상태로 표현할 방법도 없기 때문. */
    ESP_LOGI(TAG, "tasks started, FSM owns the rest");
}
