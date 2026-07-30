#include "lidar_parser.h"
#include <stddef.h>

/* 패킷 내 바이트 인덱스 (TOFSense Frame0, 16B) */
enum {
    IDX_HEADER      = 0,
    IDX_FUNC_MARK   = 1,
    IDX_SYSTIME     = 4,     /* 4바이트 */
    IDX_DIST_LOW    = 8,
    IDX_DIST_MID    = 9,
    IDX_DIST_HIGH   = 10,
    IDX_STATUS      = 11,
    IDX_STRENGTH_LO = 12,
    IDX_STRENGTH_HI = 13,
    IDX_PRECISION   = 14,
    IDX_CHECKSUM    = 15
};

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

void lidar_parser_peek_raw(const uint8_t *buf, lidar_sample_t *out)
{
    if ((buf != NULL) && (out != NULL))
    {
        out->raw_mm = ((uint32_t)buf[IDX_DIST_LOW]) |
                      (((uint32_t)buf[IDX_DIST_MID])  << 8U) |
                      (((uint32_t)buf[IDX_DIST_HIGH]) << 16U);

        out->device_time_ms = ((uint32_t)buf[IDX_SYSTIME])              |
                              (((uint32_t)buf[IDX_SYSTIME + 1U]) << 8U)  |
                              (((uint32_t)buf[IDX_SYSTIME + 2U]) << 16U) |
                              (((uint32_t)buf[IDX_SYSTIME + 3U]) << 24U);

        out->signal_strength = (uint16_t)(((uint16_t)buf[IDX_STRENGTH_LO]) |
                                          (((uint16_t)buf[IDX_STRENGTH_HI]) << 8U));

        out->dis_status      = buf[IDX_STATUS];
        out->range_precision = buf[IDX_PRECISION];
        out->confidence      = LIDAR_CONF_INVALID;
    }
}

bool lidar_parser_validate(const uint8_t *buf, lidar_sample_t *out)
{
    bool is_valid = false;

    if ((buf != NULL) && (out != NULL))
    {
        const uint8_t expected_chk = calc_checksum(buf, (uint8_t)(LIDAR_PACKET_SIZE - 1U));
        const uint8_t actual_chk   = (uint8_t)buf[IDX_CHECKSUM];

        if (expected_chk == actual_chk)
        {
            lidar_sample_t s = {0};
            lidar_parser_peek_raw(buf, &s);

            /* 🚨 문서 충돌 (2026-07-29 확인, 미해결):
             *   Datasheet V2.0 Table 5 : 0 = invalid, 1 = valid
             *   User Manual  Table 2   : status=0 예제 프레임을 정상값으로 제시
             *   실측으로 판별되기 전까지는 둘 다 통과시키고 status 는 원본 그대로 넘긴다. */
            if ((s.dis_status == 0U) || (s.dis_status == 1U))
            {
                if ((s.raw_mm >= LIDAR_MIN_RANGE_MM) && (s.raw_mm <= LIDAR_MAX_RANGE_MM))
                {
                    if (s.signal_strength == 0U)
                    {
                        /* 강도 0 → 무효, is_valid 그대로 false */
                    }
                    else if ((s.dis_status == 1U) || (s.signal_strength < LIDAR_MIN_INTENSITY))
                    {
                        s.confidence = LIDAR_CONF_LOW;
                        *out = s;
                        is_valid = true;
                    }
                    else
                    {
                        s.confidence = LIDAR_CONF_HIGH;
                        *out = s;
                        is_valid = true;
                    }
                }
            }
        }
    }

    return is_valid;
}