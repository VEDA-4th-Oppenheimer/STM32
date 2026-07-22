#include "lidar_parser.h"

/* 패킷 내 바이트 인덱스 */
enum {
    IDX_HEADER    = 0,
    IDX_FUNC_MARK = 1,
    IDX_DIST_LOW  = 8,
    IDX_DIST_MID  = 9,
    IDX_DIST_HIGH = 10,
    IDX_STATUS    = 11,
    IDX_CHECKSUM  = 15,
};

#define LIDAR_MAX_RANGE_MM    10000   /* 유효 거리 상한 (10m) */

static uint8_t calc_checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return sum;
}

bool lidar_parser_is_header(uint8_t byte)
{
    return byte == LIDAR_HEADER;
}

bool lidar_parser_is_func_mark(uint8_t byte)
{
    return byte == LIDAR_FUNC_MARK;
}

bool lidar_parser_validate(const uint8_t *buf, uint32_t *out_raw_mm)
{
    /* 1. 체크섬 불일치 → 깨진 패킷 버림 */
    if (calc_checksum(buf, LIDAR_PACKET_SIZE - 1) != buf[IDX_CHECKSUM])
    {
        return false;
    }

    /* 2. 진짜 무효한 상태만 걸러냄 (Nooploop 데이터시트 Distance Status 기준)
     *    0  = 정상 측정
     *    1  = 표준편차 15mm 초과 (경고성, 사용 가능한 데이터)
     *    2  = 신호 강도 부족
     *    4  = 위상(Phase) 경계 초과
     *    5  = HW 또는 VCSEL 결함
     *    7  = 위상 불일치
     *    8  = 내부 알고리즘 언더플로우
     *    14 = 측정 자체가 무효
     * → 0, 1만 통과시키고 나머지는 실제 무효 데이터로 간주해 버림 */
    uint8_t status = buf[IDX_STATUS];
    if (status != 0 && status != 1)
    {
        return false;
    }

    /* 3. 거리 데이터 추출 (Little-Endian 24-bit) */
    uint32_t raw_mm = ((uint32_t)buf[IDX_DIST_LOW]) |
                       ((uint32_t)buf[IDX_DIST_MID]  << 8) |
                       ((uint32_t)buf[IDX_DIST_HIGH] << 16);

    /* 4. 유효 범위 체크 (10m 이내만 인정) */
    if (raw_mm >= LIDAR_MAX_RANGE_MM)
    {
        return false;
    }

    *out_raw_mm = raw_mm;
    return true;
}