/** @file bsp_power.h  @brief 배터리 ADC(분압 5.545:1)와 5V 존재 감지. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void bsp_power_init(void);
uint16_t bsp_batt_mv(void);     /* 3S 합산 전압 */
bool bsp_main5v_present(void);
