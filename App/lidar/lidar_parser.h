#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_HEADER          0x57
#define LIDAR_FUNC_MARK       0x00
#define LIDAR_PACKET_SIZE     16

/**
 * @brief 헤더 바이트(0x57)가 맞는지 검사
 */
bool lidar_parser_is_header(uint8_t byte);

/**
 * @brief Function Mark 바이트(0x00)가 맞는지 검사
 */
bool lidar_parser_is_func_mark(uint8_t byte);

/**
 * @brief 완성된 16바이트 패킷을 검증(체크섬 + 센서 상태 + 유효 범위)하고
 *        통과하면 raw 거리값(mm)을 out_raw_mm에 담아 반환
 * @param buf        16바이트 패킷 버퍼
 * @param out_raw_mm 파싱 성공 시 raw 거리값(mm)이 저장될 포인터
 * @return true  = 유효한 패킷, out_raw_mm에 값 저장됨
 *         false = 체크섬 불일치 / 센서 에러 상태 / 범위 밖 → out_raw_mm 미변경
 */
bool lidar_parser_validate(const uint8_t *buf, uint32_t *out_raw_mm);

#endif /* LIDAR_PARSER_H */