/**
 * @file bsp_heater.h
 * @brief 열선 출력. 여기만 HEAT_* 핀을 만진다.
 *
 * 안전 규칙
 *   1. bsp_heater_init_safe()는 app_main 맨 처음에 부른다
 *   2. ARM이 LOW면 채널을 올려도 전류가 흐르지 않는다(하드웨어 2중 차단)
 *   3. 동시에 켤 수 있는 채널은 1개(CFG_HEAT_MAX_ACTIVE)
 *   4. NURA_DISABLE_HEATER_OUTPUT이 켜지면 전부 no-op (벤치 빌드)
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

void bsp_heater_init_safe(void);   /* 전 채널 + ARM을 LOW로 고정 */
void bsp_heater_arm(bool on);
bool bsp_heater_is_armed(void);
bool bsp_heater_set(uint8_t ch, bool on);   /* ARM이 꺼져 있거나 다른 채널이 켜져 있으면 false */
void bsp_heater_all_off(void);
