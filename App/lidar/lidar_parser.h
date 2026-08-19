/* ============================================================================
 *  lidar_parser.h  --  TOFSense-F2P NLink Frame0 파서
 * ----------------------------------------------------------------------------
 *  담당: 송영빈 (원 구현) / 이현우 (v5 필드 확장)
 *
 *  핵심: 파서는 **거르지 않는다.** 체크섬만 보고, 나머지는 프레임에 실려온 값을
 *    그대로 넘긴다. 유효성 판정(거리 범위·dis_status·신호세기)은 RPi 데몬 몫이다.
 *
 *    왜: 판정 기준이 바뀌면 이미 찍어둔 스캔을 다시 해석할 수 있어야 한다.
 *    펌웨어가 미리 버리면 그 점은 영영 복구되지 않는다. 실제로 1축 브링업에서
 *    거리 상한 필터 때문에 먼 벽이 잘려나간 적이 있고, 그때 쓰던 상한도
 *    F2P 사양(25m)이 아니었다.
 *
 *  NLink Frame0 (16바이트) 레이아웃 — 데이터시트 대조 확인분:
 *    [0]      header 0x57
 *    [1]      function mark 0x00
 *    [2]      reserved
 *    [3]      id
 *    [4..7]   system time (u32, LE)   라이다 자체 시계(ms)
 *    [8..10]  distance    (u24, LE)   mm
 *    [11]     distance status         1=valid, 0=invalid  ※ 아래 참조
 *    [12..13] signal strength (u16, LE)
 *    [14]     range precision         F2P 는 미지원(0xFF 고정)
 *    [15]     checksum = [0..14] 바이트 합의 하위 8비트
 *
 *  주의: dis_status 의미: 데이터시트는 1=valid 인데 매뉴얼 예제는 반대로 적혀 있다.
 *    1축 브링업 실측에서 유효점 359/359 가 전부 1 이었으므로 **데이터시트가
 *    맞다**. 다만 여기서 걸러내지는 않는다 — 값만 그대로 올린다.
 * ==========================================================================*/
#ifndef LIDAR_PARSER_H
#define LIDAR_PARSER_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_HEADER          0x57U
#define LIDAR_FUNC_MARK       0x00U
#define LIDAR_PACKET_SIZE     16U

/* 프레임에서 뽑아낸 원본 값. 가공하지 않는다. */
typedef struct {
    uint32_t device_time_ms;    /* 라이다 자체 시계          */
    uint32_t d_mm;              /* 24비트라 uint32 로 받는다 */
    uint16_t signal_strength;
    uint8_t  dis_status;
    uint8_t  range_precision;
} lidar_frame_t;

/* 헤더/펑션마크 바이트 판별 (수신 상태머신 동기화용) */
bool lidar_parser_is_header(uint8_t byte);
bool lidar_parser_is_func_mark(uint8_t byte);

/* 완성된 16바이트 패킷의 체크섬을 검증하고 필드를 out 에 채운다.
 *   반환 true  = 체크섬 통과, out 유효
 *        false = 체크섬 불일치 (out 미변경)
 * 체크섬 외의 이유로는 실패하지 않는다 — 거르는 건 데몬 몫. */
bool lidar_parser_parse(const uint8_t *buf, lidar_frame_t *out);

#endif /* LIDAR_PARSER_H */
