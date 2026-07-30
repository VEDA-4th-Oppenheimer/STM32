#include "lidar_parser.h"
#include <stddef.h>

/* 패킷 내 바이트 인덱스 (TOFSense Frame0, 16B)
 *
 *   0     header 0x57
 *   1     function mark 0x00
 *   2     reserved
 *   3     id
 *   4~7   system time (uint32 LE)     ← v5 에서 사용 시작
 *   8~10  distance (uint24 LE, mm)
 *   11    distance status
 *   12~13 signal strength (uint16 LE) ← v5 에서 사용 시작
 *   14    range precision             ← v5 에서 사용 시작
 *   15    sum check
 */
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

/* ---- 유효 거리 범위 ---------------------------------------------------------
 *  ⚠️ 하한이 없으면 raw=0(범위초과/무반사)이 "거리 0mm" 로 통과해
 *    포인트클라우드 원점에 가짜 점이 쌓인다(실측: .pcd 꼬리의 0.0 -0.0 점들).
 *
 *  TOFSense-F2 P 실측 스펙 (Datasheet V2.0 Table 1, 2026-07-29 확인):
 *   Typical Ranging Distance : 0.05 ~ 25.0 m   ← 50m 는 F2 PH 모델 값이었음
 *   Blind Area               : 5 cm
 *   Accuracy                 : ±3.0cm @[0.05,25]m
 *   Std Dev                  : <1.0cm @[0.05,10]m, <6.0cm @[10,25]m
 *   Wavelength               : 905 nm (F2 계열)
 *
 * ⚠️ 범위를 넘으면 센서가 거리 0 을 출력한다(User Manual Q9).
 *   따라서 하한 50mm 는 "blind area" 이자 "범위 초과(0) 걸러내기" 역할을 겸한다. */
#define LIDAR_MIN_RANGE_MM       50U   /* 0.05 m — blind area, 0(범위초과) 배제 */
#define LIDAR_MAX_RANGE_MM    25000U   /* 25 m   — F2 P 스펙 상한 */

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

        out->device_time_ms = ((uint32_t)buf[IDX_SYSTIME])            |
                              (((uint32_t)buf[IDX_SYSTIME + 1U]) << 8U)  |
                              (((uint32_t)buf[IDX_SYSTIME + 2U]) << 16U) |
                              (((uint32_t)buf[IDX_SYSTIME + 3U]) << 24U);

        out->signal_strength = (uint16_t)(((uint16_t)buf[IDX_STRENGTH_LO]) |
                                          (((uint16_t)buf[IDX_STRENGTH_HI]) << 8U));

        out->dis_status      = buf[IDX_STATUS];
        out->range_precision = buf[IDX_PRECISION];
    }
}

bool lidar_parser_validate(const uint8_t *buf, uint32_t sys_time_ms, lidar_parsed_data_t *out_data)
{
    bool is_valid = false;

    /* misra-c2012-2.7 회피: 사용되지 않는 매개변수 명시적 캐스팅 */
    ((void)sys_time_ms);

    if ((buf != NULL) && (out != NULL))
    {
        /* 1. 체크섬 검사 (양쪽 모두 명시적 uint8_t 캐스팅 적용) */
        const uint8_t expected_chk = calc_checksum(buf, (uint8_t)(LIDAR_PACKET_SIZE - 1U));
        const uint8_t actual_chk   = (uint8_t)buf[IDX_CHECKSUM];

        if (expected_chk == actual_chk)
        {
            lidar_sample_t s = {0};
            lidar_parser_peek_raw(buf, &s);

            /* 2. Distance Status 검사
             *
             * 🚨 문서 충돌 (2026-07-29 확인, 미해결):
             *   Datasheet V2.0 Table 5 : 0 = **invalid**, 1 = **valid**
             *   User Manual  Table 2   : 예제 프레임이 status=0 인데 거리
             *                            2.221m 를 정상값으로 제시
             *   둘이 정반대다. status==1 만 통과시켰다가 실제로 0 이 정상이면
             *   **모든 점이 사라진다**. 반대로 지금처럼 둘 다 통과시키면
             *   무효 측정이 섞일 수 있다.
             *
             *   → 지금은 **둘 다 통과**시키고 원본 status 를 v5 로 상행한다.
             *     데몬이 status 분포를 집계하므로, 실측 1회로 판별된다:
             *       거의 전부 1  → Datasheet 가 맞음 → 여기를 (==1) 로 좁힌다
             *       거의 전부 0  → Manual 이 맞음   → 현행 유지
             *       섞여 있음    → 값별 거리 품질을 비교해 결정
             *     판별 후 이 주석과 조건을 반드시 확정할 것. */
            if ((s.dis_status == 0U) || (s.dis_status == 1U))
            {
                /* 3. 유효 범위 체크 (0.05 ~ 25m). 벗어나면 bad_pkts 로 집계. */
                if ((s.raw_mm >= LIDAR_MIN_RANGE_MM) && (s.raw_mm <= LIDAR_MAX_RANGE_MM))
                {
                    *out = s;
                    is_valid = true;
                }
            }
        }

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