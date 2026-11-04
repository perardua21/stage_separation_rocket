/** @file bsp_storage.h  @brief microSD 로깅. 비행 중 블로킹 금지. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* 규칙: append()는 RAM 버퍼에 복사만 한다. 실제 쓰기는 service()가
 * 한 번에 한 블록씩 처리한다. 협력형이 아니라 FreeRTOS지만, SD가 바쁠 때
 * 로거 태스크가 오래 잡고 있으면 같은 코어의 FSM이 밀리기 때문. */
bool bsp_storage_init(void);
bool bsp_storage_append(const uint8_t *rec, uint16_t len);
void bsp_storage_service(void);
void bsp_storage_flush_and_close(void);
bool bsp_storage_healthy(void);
uint32_t bsp_storage_dropped(void);
