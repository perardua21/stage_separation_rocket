/** @file bsp_sensors.h  @brief BNO085(SPI) / BMP390(I2C) 접근. */
#pragma once
#include "flight_types.h"

/* BNO085: 자체 융합으로 쿼터니언을 준다. 별도 EKF를 돌리지 않아도 된다.
 * ⚠ 가속도 레인지가 ±8 g다. 시뮬 최대 가속도가 약 8.4 G라서 발사 순간
 *   포화될 수 있다(docs/04-open-items.md 2번). 발사 감지("2 G 이상")는
 *   포화해도 되지만, 가속도 적분 속도는 과소평가된다. */
bool bsp_bno_init(void);
bool bsp_bno_read(imu_sample_t *out);
void bsp_bno_reset(void);

bool bsp_bmp_init(void);
bool bsp_bmp_read(baro_sample_t *out);

/* 지상 기준기압. FS_INIT에서 10 s 평균으로 잡고 비휘발성에 저장한다.
 * 비행 중 리셋 후 현재 고도를 알려면 이 값이 필요하기 때문. */
bool bsp_baro_set_ground_ref(float pa);
float bsp_baro_ground_ref(void);
float bsp_baro_to_agl(float pa);
