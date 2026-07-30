#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_HEADER          0x57U
#define LIDAR_FUNC_MARK       0x00U
#define LIDAR_PACKET_SIZE     16U

/**
 * @brief 헤더 바이트(0x57)가 맞는지 검사
 */
bool lidar_parser_is_header(uint8_t byte);

/**
 * @brief Function Mark 바이트(0x00)가 맞는지 검사
 */
bool lidar_parser_is_func_mark(uint8_t byte);

/* ---------------------------------------------------------------------------
 *  파싱 결과 (protocol.h v5 로 상행할 원시 필드)
 *
 *  ⚠️ 전부 **센서 원본값 그대로** 담는다. 정규화·판정은 상위(데몬/캘리브).
 *    특히 signal_strength 는 calibrated reflectivity 가 아니므로 재질 판별에
 *    단독 사용 금지(PointCloud 계획서 §6.2).
 *
 *  ⚠️ 바이트 배치는 TOFSense Frame0(16B) 표준 기준이다. dist(8~10)·status(11)·
 *    checksum(15) 는 실기로 검증됐고, 나머지(4~7 systime / 12~13 strength /
 *    14 precision)는 같은 표준 배치를 따른 것이므로 브링업 때 실측 확인할 것:
 *      - device_time_ms 가 샘플마다 ~10ms 씩 단조 증가하는가
 *      - 손을 가까이/멀리 할 때 signal_strength 가 변하는가
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t raw_mm;           /* 거리 (mm)                       */
    uint32_t device_time_ms;   /* 라이다 system time 원본         */
    uint16_t signal_strength;  /* 신호세기 원본                   */
    uint8_t  dis_status;       /* 거리상태 원본                   */
    uint8_t  range_precision;  /* 거리정밀도 원본                 */
} lidar_sample_t;

/**
 * @brief 완성된 16바이트 패킷을 검증(체크섬 + 센서 상태 + 유효 범위)하고
 *        통과하면 원시 필드 전체를 out 에 담아 반환
 * @param buf 16바이트 패킷 버퍼
 * @param out 파싱 성공 시 채워질 구조체
 * @return true  = 유효한 패킷
 *         false = 체크섬 불일치 / 센서 에러 상태 / 범위 밖 → out 미변경
 * @note  검증 실패해도 dis_status·signal_strength 는 진단에 쓸 수 있으므로
 *        호출측이 필요하면 lidar_parser_peek_raw() 로 따로 꺼낸다.
 */
bool lidar_parser_validate(const uint8_t *buf, lidar_sample_t *out);

/**
 * @brief 검증 여부와 무관하게 원시 필드만 추출 (실패 패킷 진단용).
 *        체크섬 검사를 하지 않으므로 값이 깨져 있을 수 있다.
 */
void lidar_parser_peek_raw(const uint8_t *buf, lidar_sample_t *out);

#endif /* LIDAR_PARSER_H */