/* ============================================================================
 *  scan.h  --  스캔 시퀀스 (모터 스윕 + 라이다 샘플 각도 래치 + 상행)
 * ----------------------------------------------------------------------------
 *  담당: 이현우
 *
 *  motor / lidar / uart_rpi 를 엮는 "조율" 계층. 각 모듈은 자기 일만 하고,
 *  이 파일이 셋을 연결한다(모듈 간 직접 의존 없음).
 *
 *      uart_rpi ──CMD_SCAN_START──▶ scan ──▶ motor  (팬 스윕 시작)
 *                                    ▲
 *                                    └── lidar (거리 도착 콜백)
 *                                    │
 *                                    └──▶ uart_rpi (scan_point 상행)
 *
 *  ★ 2축 확장 대비: 시퀀스를 "틸트 줄마다 팬 스윕" 루프로 짜고,
 *    1축은 tilt_start == tilt_end 인 특수 케이스로 처리한다.
 *    따라서 2축 기구가 완성돼도 이 파일의 구조는 그대로 쓴다.
 * ==========================================================================*/
#ifndef SCAN_H
#define SCAN_H

#include "protocol.h"      /* struct proto_scan_start */
#include "lidar_parser.h"  /* lidar_sample_t */
#include <stdint.h>

/* 1회 초기화 (main() USER CODE 2). */
void scan_init(void);

/* 스캔 시작. uart_rpi 가 CMD_SCAN_START 수신 시 호출.
 * ss 범위/격자로 시퀀스를 시작한다. 이미 스캔 중이면 무시. */
void scan_start(const struct proto_scan_start *ss);

/* 스캔 중단 (CMD_SCAN_STOP / DISARM). 모터 정지 + 상태 초기화. */
void scan_stop(void);

/* 메인루프에서 매회 호출: 래치된 점 상행 + 스윕 진행/종료 판정.
 * ⚠️ 상행(UART 블로킹)은 반드시 여기서 — ISR 에서 하면 라이다 패킷을 놓친다. */
void scan_tick(void);

/* 라이다가 유효 프레임을 파싱한 순간 호출(ISR 문맥).
 * 여기서는 각도 래치 + 링버퍼 적재만 하고 즉시 반환한다.
 * v5: 거리 외 원시 품질 필드(status/strength/precision/device_time)도 함께 래치. */
void scan_on_lidar_sample(const lidar_sample_t *s);

/* 스캔 품질 통계 (진단용).
 *   sent    : 상행 완료한 점 수
 *   dropped : FIFO 가득 차서 버린 점 수  ← 0 이 아니면 메인루프가 밀린 것
 *
 * ⚠️ dropped 는 RPi 쪽에서 보이지 않는다. 버려진 점은 애초에 전송되지 않아
 *   STM 의 point_count 에도 안 잡히기 때문(데몬의 기대값 대조로는 검출 불가).
 *   그래서 STM 단에서 노출해야 유실을 알 수 있다.
 *   카운터는 scan_start() 마다 리셋된다. */
void scan_get_stats(uint32_t *sent, uint32_t *dropped);

/* 스캔·되감기 진행 중인가. 1 이면 메인루프에서 블로킹 작업(printf 등) 금지.
 *
 * ⚠️ 실측 근거: 1초 간격 진단 printf 가 ~10ms 블로킹인데, 그게 되감기 완료
 *   판정 순간에 걸리면 그동안 9~18 스텝(1~2도)이 더 나가 축이 과도하게 되감긴다
 *   (연속 스캔 상호상관 측정: -2도/회). 스캔 중에는 출력을 멈춰야 한다. */
uint8_t scan_is_busy(void);

/* 스캔이 방금 끝났는지 확인하고 플래그를 소비(1회성).
 * 완료 시점에 최종 통계를 한 번 출력하는 용도. 1=방금 끝남. */
uint8_t scan_take_finished(void);

#endif /* SCAN_H */
