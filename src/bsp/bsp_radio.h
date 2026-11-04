/**
 * @file bsp_radio.h
 * @brief E22-900T30D-V2 (SX1262 1 W). UART + M0/M1 모드핀 + AUX.
 *
 * 작년 SX1276 직접 제어와 달리 이 모듈은 UART로 투명 전송한다.
 * M0/M1에 10k 풀다운이 있어 기본이 00(전송 모드)이다.
 * 파라미터를 바꾸려면 11(설정 모드)로 올렸다가 되돌려야 한다.
 *
 * ⚠ 1 W 출력이다. 주파수·공중선전력·듀티를 국내 규정과 대회 규정에
 *   맞춰 확인할 것(docs/04-open-items.md 5번).
 * ⚠ 보드 두 장이 같이 쏜다. 송신 슬롯을 나눠야 서로 깨지지 않는다.
 *   부스터 0~100 ms, 서스테이너 100~200 ms (200 ms 주기 기준).
 */
#pragma once
#include "flight_types.h"

bool bsp_radio_init(role_t role);
bool bsp_radio_send(const uint8_t *buf, uint16_t len);
int  bsp_radio_recv(uint8_t *buf, uint16_t max_len);   /* 받은 바이트 수, 없으면 0 */
bool bsp_radio_busy(void);                             /* AUX 핀 */
uint32_t bsp_radio_slot_offset_ms(role_t role);
