#include "lidar_parser.h"
#include <stddef.h>

/* 패킷 내 바이트 인덱스 명세 (NLink / TOFSense F2P 16바이트 기준) */
enum {
    IDX_HEADER           = 0,
    IDX_FUNC_MARK        = 1,
    IDX_RESERVED         = 2,
    IDX_MODULE_ID        = 3,
    IDX_TIME_LOW         = 4,   /* 시스템 타임 (Byte 4 ~ 7) */
    IDX_DIST_LOW         = 8,   /* 거리 (Byte 8 ~ 10) */
    IDX_DIST_MID         = 9,
    IDX_DIST_HIGH        = 10,
    IDX_STATUS           = 11,  /* 거리 상태 (Byte 11) */
    IDX_INTENSITY_LOW    = 12,  /* 신호 강도 (Byte 12 ~ 13) */
    IDX_INTENSITY_HIGH   = 13,
    IDX_PRECISION        = 14,  /* 정밀도 (Byte 14) */
    IDX_CHECKSUM         = 15   /* 체크섬 (Byte 15) */
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

bool lidar_parser_validate(const uint8_t *buf, uint32_t sys_time_ms, lidar_parsed_data_t *out_data)
{
    bool is_valid = false;

    /* misra-c2012-2.7 회피: 사용되지 않는 매개변수 명시적 캐스팅 */
    ((void)sys_time_ms);

    if ((buf != NULL) && (out_data != NULL))
    {
        /* 1. 체크섬 검사 (처음 15바이트의 합 == 마지막 15번째 바이트) */
        const uint8_t expected_chk = calc_checksum(buf, (uint8_t)(LIDAR_PACKET_SIZE - 1U));
        const uint8_t actual_chk   = (uint8_t)buf[IDX_CHECKSUM];

        if (expected_chk == actual_chk)
        {
            /* 2. 각 필드 데이터 추출 */
            const uint8_t status = buf[IDX_STATUS];

            /* 거리 (24-bit Little-Endian) */
            const uint32_t raw_mm = ((uint32_t)buf[IDX_DIST_LOW]) |
                                    (((uint32_t)buf[IDX_DIST_MID])  << 8U) |
                                    (((uint32_t)buf[IDX_DIST_HIGH]) << 16U);

            /* 신호 강도 Intensity (16-bit Little-Endian) */
            const uint16_t intensity = ((uint16_t)buf[IDX_INTENSITY_LOW]) |
                                        (((uint16_t)buf[IDX_INTENSITY_HIGH]) << 8U);

            /* 센서 내부 시스템 타임 (32-bit Little-Endian) */
            const uint32_t sensor_time = ((uint32_t)buf[IDX_TIME_LOW])       |
                                         (((uint32_t)buf[IDX_TIME_LOW + 1U]) << 8U)  |
                                         (((uint32_t)buf[IDX_TIME_LOW + 2U]) << 16U) |
                                         (((uint32_t)buf[IDX_TIME_LOW + 3U]) << 24U);

            /* 3. 추출값 구조체에 대입 */
            out_data->raw_mm         = raw_mm;
            out_data->intensity      = intensity;
            out_data->status         = status;
            out_data->system_time_ms = (uint16_t)(sensor_time & 0xFFFFU);

            /* 4. 신뢰도(Confidence) 평가 로직 (misra-c2012-12.1 준수: 괄호 명시) */
            if (((status != 0U) && (status != 1U)))
            {
                out_data->confidence = LIDAR_CONF_INVALID;
            }
            else if (((raw_mm < LIDAR_MIN_RANGE_MM) || (raw_mm > LIDAR_MAX_RANGE_MM)))
            {
                out_data->confidence = LIDAR_CONF_INVALID;
            }
            else if (intensity == 0U)
            {
                out_data->confidence = LIDAR_CONF_INVALID;
            }
            else if (((status == 1U) || (intensity < LIDAR_MIN_INTENSITY)))
            {
                out_data->confidence = LIDAR_CONF_LOW;
                is_valid = true;
            }
            else
            {
                out_data->confidence = LIDAR_CONF_HIGH;
                is_valid = true;
            }
        }
    }

    return is_valid;
}