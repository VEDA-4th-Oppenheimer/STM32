#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_HEADER          0x57U
#define LIDAR_FUNC_MARK       0x00U
#define LIDAR_PACKET_SIZE     16U

/* 신호 강도 문턱값 및 범위 정의 (필요시 센서 스펙에 따라 조정) */
#define LIDAR_MIN_INTENSITY   10U     /* 노이즈 필터링 문턱값 (이 값 미만은 노이즈 간주) */
#define LIDAR_MIN_RANGE_MM    100U    /* 유효 하한 (10cm) */
#define LIDAR_MAX_RANGE_MM    10000U  /* 유효 상한 (10m) */

/**
 * @brief 신뢰도 (Confidence Level) 정의
 */
typedef enum {
    LIDAR_CONF_INVALID = 0U,  /* 무효 데이터 (체크섬/상태 이상/노이즈) */
    LIDAR_CONF_LOW     = 1U,  /* 신호 강도 약함 또는 범위 경계값 (주의) */
    LIDAR_CONF_HIGH    = 2U   /* 신뢰성 높은 정상 데이터 */
} lidar_confidence_t;

/**
 * @brief 파싱된 라이다 측정 데이터 구조체
 */
typedef struct {
    uint32_t           raw_mm;         /* 거리 데이터 (mm) */
    uint16_t           intensity;      /* 신호 강도 (Signal Intensity) */
    uint16_t           system_time_ms; /* 센서 자체/시스템 타임스탬프 (ms) */
    uint8_t            status;         /* Distance Status */
    lidar_confidence_t confidence;     /* 종합 평가된 신뢰도 */
} lidar_parsed_data_t;

/**
 * @brief 헤더 바이트(0x57) 검사
 */
bool lidar_parser_is_header(uint8_t byte);

/**
 * @brief Function Mark 바이트(0x00) 검사
 */
bool lidar_parser_is_func_mark(uint8_t byte);

/**
 * @brief 16바이트 패킷을 파싱하여 거리, 신호강도, 타임스탬프, 신뢰도를 추출
 * @param buf       16바이트 패킷 버퍼
 * @param sys_time_ms 수신 시점의 호스트/STM32 타임스탬프 (HAL_GetTick() 값)
 * @param out_data  파싱 결과물이 저장될 구조체 포인터
 * @return true = valid 패킷, false = 체크섬 실패 또는 invalid
 */
bool lidar_parser_validate(const uint8_t *buf, uint32_t sys_time_ms, lidar_parsed_data_t *out_data);

#endif /* LIDAR_PARSER_H */