/* ============================================================================
 *  lidar_parser.c  --  TOFSense-F2P NLink Frame0 파서 구현
 *  담당: 송영빈 (원 구현) / 이현우 (v5 필드 확장)
 *  계약은 lidar_parser.h 상단 참조.
 * ==========================================================================*/
#include "lidar_parser.h"
#include <stddef.h>

/* 패킷 내 바이트 인덱스 (데이터시트 Frame0 레이아웃) */
enum {
    IDX_HEADER      = 0,
    IDX_FUNC_MARK   = 1,
    IDX_TIME_0      = 4,     /* system time     u32 LE (4..7)   */
    IDX_DIST_LOW    = 8,     /* distance        u24 LE (8..10)  */
    IDX_DIST_MID    = 9,
    IDX_DIST_HIGH   = 10,
    IDX_STATUS      = 11,
    IDX_SIGNAL_LOW  = 12,    /* signal strength u16 LE (12..13) */
    IDX_SIGNAL_HIGH = 13,
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

bool lidar_parser_parse(const uint8_t *buf, lidar_frame_t *out)
{
    bool is_valid = false;

    if ((buf != NULL) && (out != NULL))
    {
        /* 체크섬은 전송 무결성 검사라 여기서 본다.
         * 그 외의 판정(거리 범위 / dis_status / 신호세기)은 하지 않는다 —
         * 펌웨어가 버리면 그 점은 영영 복구되지 않기 때문. */
        const uint8_t expected_chk = calc_checksum(buf, (uint8_t)(LIDAR_PACKET_SIZE - 1U));
        const uint8_t actual_chk   = buf[IDX_CHECKSUM];

        if (expected_chk == actual_chk)
        {
            out->device_time_ms = ((uint32_t)buf[IDX_TIME_0])               |
                                  (((uint32_t)buf[IDX_TIME_0 + 1]) <<  8U) |
                                  (((uint32_t)buf[IDX_TIME_0 + 2]) << 16U) |
                                  (((uint32_t)buf[IDX_TIME_0 + 3]) << 24U);

            out->d_mm = ((uint32_t)buf[IDX_DIST_LOW])           |
                        (((uint32_t)buf[IDX_DIST_MID])  <<  8U) |
                        (((uint32_t)buf[IDX_DIST_HIGH]) << 16U);

            {
                const uint32_t sig = ((uint32_t)buf[IDX_SIGNAL_LOW]) |
                                     (((uint32_t)buf[IDX_SIGNAL_HIGH]) << 8U);
                out->signal_strength = (uint16_t)sig;
            }

            out->dis_status      = buf[IDX_STATUS];
            out->range_precision = buf[IDX_PRECISION];

            is_valid = true;
        }
    }

    return is_valid;
}
