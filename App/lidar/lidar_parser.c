#include "lidar_parser.h"
#include <stddef.h>

/* 패킷 내 바이트 인덱스 */
enum {
    IDX_HEADER    = 0,
    IDX_FUNC_MARK = 1,
    IDX_DIST_LOW  = 8,
    IDX_DIST_MID  = 9,
    IDX_DIST_HIGH = 10,
    IDX_STATUS    = 11,
    IDX_CHECKSUM  = 15
};

#define LIDAR_MAX_RANGE_MM    10000U   /* 유효 거리 상한 (10m) */

static uint8_t calc_checksum(const uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < len; i++)
    {
        sum += buf[i];
    }
    return sum;
}

bool lidar_parser_is_header(uint8_t byte)
{
    return (byte == (uint8_t)LIDAR_HEADER);
}

bool lidar_parser_is_func_mark(uint8_t byte)
{
    return (byte == (uint8_t)LIDAR_FUNC_MARK);
}

bool lidar_parser_validate(const uint8_t *buf, uint32_t *out_raw_mm)
{
    bool is_valid = false;

    if ((buf != NULL) && (out_raw_mm != NULL))
    {
        /* 1. 체크섬 검사 (양쪽 모두 명시적 uint8_t 캐스팅 적용) */
        const uint8_t expected_chk = calc_checksum(buf, (uint8_t)(LIDAR_PACKET_SIZE - 1U));
        const uint8_t actual_chk   = (uint8_t)buf[IDX_CHECKSUM];

        if (expected_chk == actual_chk)
        {
            /* 2. Distance Status 검사 (0: 정상, 1: 경고성 사용가능) */
            const uint8_t status = buf[IDX_STATUS];
            if ((status == 0U) || (status == 1U))
            {
                /* 3. 거리 데이터 추출 (Little-Endian 24-bit) */
                const uint32_t raw_mm = ((uint32_t)buf[IDX_DIST_LOW]) |
                                        (((uint32_t)buf[IDX_DIST_MID])  << 8U) |
                                        (((uint32_t)buf[IDX_DIST_HIGH]) << 16U);

                /* 4. 유효 범위 체크 (10m 이내만 인정) */
                if (raw_mm < LIDAR_MAX_RANGE_MM)
                {
                    *out_raw_mm = raw_mm;
                    is_valid = true;
                }
            }
        }
    }

    return is_valid;
}