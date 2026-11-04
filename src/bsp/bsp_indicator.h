/** @file bsp_indicator.h  @brief 부저 + LED. 상태를 소리와 빛으로. */
#pragma once
#include <stdint.h>

typedef enum {
    BZ_NONE = 0,
    BZ_INIT_DONE,      /* 초기화 완료 */
    BZ_ROLE_BOOSTER,   /* 짧게 1회 — 역할 확인용 */
    BZ_ROLE_SUSTAINER, /* 짧게 2회 */
    BZ_ARMED,          /* 긴 경고음 */
    BZ_TRANSITION,
    BZ_BEACON,         /* 착지 후 회수용 반복 */
} buzzer_pattern_t;

typedef enum {
    PANIC_STAGE_ID = 2,   /* 비프 2회 반복 */
    PANIC_ALLOC    = 3,
    PANIC_TASK     = 4,
    PANIC_SENSOR   = 5,
    PANIC_SD       = 6,
} panic_code_t;

void bsp_indicator_init(void);
void bsp_buzzer_pattern(buzzer_pattern_t p);
void bsp_led(bool red, bool green);
void bsp_indicator_panic(panic_code_t code);  /* 돌아오지 않는다 */
