#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_HEADER          0x57U
#define LIDAR_FUNC_MARK       0x00U
#define LIDAR_PACKET_SIZE     16U

/* 신호 강도 문턱값 및 유효 거리 범위
 * TOFSense-F2 P Datasheet V2.0 Table 1 실측 확인 (2026-07-29):
 *   Typical Ranging Distance : 0.05 ~ 25.0 m, Blind Area: 5cm
 * 하한은 blind area이자 범위초과(raw=0) 배제 역할을 겸함. */
#define LIDAR_MIN_INTENSITY   10U      /* 노이즈 필터링 문턱값 */
#define LIDAR_MIN_RANGE_MM    50U      /* 0.05 m */
#define LIDAR_MAX_RANGE_MM    25000U   /* 25 m */

typedef enum {
    LIDAR_CONF_INVALID = 0U,  /* 무효 데이터 (체크섬/상태 이상/노이즈) */
    LIDAR_CONF_LOW     = 1U,  /* 신호 강도 약함 또는 경계값 (주의) */
    LIDAR_CONF_HIGH    = 2U   /* 신뢰성 높은 정상 데이터 */
} lidar_confidence_t;

/* ---------------------------------------------------------------------------
 *  파싱 결과 — 센서 원본값 + 종합 판정된 confidence
 *  ⚠️ signal_strength 는 calibrated reflectivity 가 아니므로
 *    재질 판별에 단독 사용 금지(PointCloud 계획서 §6.2).
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t           raw_mm;           /* 거리 (mm)                 */
    uint32_t           device_time_ms;   /* 라이다 system time 원본   */
    uint16_t           signal_strength;  /* 신호세기 원본             */
    uint8_t            dis_status;       /* 거리상태 원본             */
    uint8_t            range_precision;  /* 거리정밀도 원본           */
    lidar_confidence_t confidence;       /* 종합 평가된 신뢰도        */
} lidar_sample_t;

bool lidar_parser_is_header(uint8_t byte);
bool lidar_parser_is_func_mark(uint8_t byte);

/**
 * @brief 검증 여부와 무관하게 원시 필드만 추출 (실패 패킷 진단용).
 *        체크섬 검사를 하지 않으므로 값이 깨져 있을 수 있다.
 */
void lidar_parser_peek_raw(const uint8_t *buf, lidar_sample_t *out);

/**
 * @brief 16바이트 패킷을 검증(체크섬 + 센서 상태 + 유효 범위 + 강도)하고
 *        통과하면 원시 필드 + confidence 를 out 에 담아 반환
 * @return true  = 유효한 패킷 (out 갱신됨)
 *         false = 체크섬 불일치 / 상태 이상 / 범위 밖 / 강도 0 → out 미변경
 * @note   실패해도 dis_status 등은 진단에 쓸 수 있으니 필요하면
 *         lidar_parser_peek_raw() 로 따로 꺼낸다.
 */
bool lidar_parser_validate(const uint8_t *buf, lidar_sample_t *out);

#endif /* LIDAR_PARSER_H */