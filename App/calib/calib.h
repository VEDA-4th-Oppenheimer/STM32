#ifndef CALIB_H
#define CALIB_H

#include <stdint.h>

/**
 * @brief 캘리브레이션 필터 상태 초기화
 */
void calib_reset(void);

/**
 * @brief raw 거리값(mm)에 오프셋 보정 + EMA 필터를 적용하고 결과를 반환
 * @param raw_mm 센서로부터 얻은 보정 전 원본 거리값 (mm)
 * @return 보정/필터링된 거리값 (mm)
 */
uint16_t calib_process_distance(uint32_t raw_mm);

/**
 * @brief 가장 최근에 계산된 보정/필터링 거리값 반환 (mm)
 */
uint16_t calib_get_distance_mm(void);

#endif /* CALIB_H */