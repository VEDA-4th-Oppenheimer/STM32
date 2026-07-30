/* ============================================================================
 *  scan.h  --  2축 스캔 시퀀서
 * ----------------------------------------------------------------------------
 *  담당: 이현우
 *
 *  ★ 메인루프에서 도는 시퀀서다. ISR 이 아니다.
 *    그래서 여기서는 블로킹 I2C(엔코더)도 HAL_Delay 도 안전하다.
 *    반대로 motor 계층(App/motor)은 ISR 에서 펄스만 내고 아무 판단도 하지
 *    않는다. 이전 구현이 시퀀서를 ISR 안에 두는 바람에 ISR 에서 HAL_Delay·
 *    printf·I2C 를 하게 됐고 데드락과 라이다 프레임 유실을 낳았다.
 *
 *  스캔 구조 (천장 마운트 2축):
 *    틸트 = 빠른 축. 한 줄이 -90 -> +90 (한쪽 벽 -> 바닥 -> 반대쪽 벽).
 *    팬   = 느린 축. 줄마다 step_ddeg 만큼. 스윕 동안은 정지.
 *    줄마다 틸트 방향을 뒤집는다(serpentine) — 되감기가 필요 없어진다.
 *
 *  각도는 전부 **기구각**이다. 계약 좌표계 변환은 RPi 데몬 몫
 *  (protocol.h §4). 펌웨어는 모터가 어디 있는지만 정직하게 보고한다.
 *
 *  엔코더 사용처 (protocol.h §4 와 동일):
 *    ① 홈 확립 — 절대 엔코더라 구동 없이 읽기 1회
 *    ② 틸트 줄 끝(±90) 대조 — 방향 전환이 탈조가 나는 자리라 재영점
 *    ③ 팬 줄 시작 대조 — 보정 없이 검증만 (보정 이동이 새 실패 모드가 됨)
 * ==========================================================================*/
#ifndef SCAN_H
#define SCAN_H

#include "main.h"
#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

/* --- 엔코더 대조 임계 -----------------------------------------------------
 *  MT6701 해상도가 0.022도, 1 스텝이 0.1125도 이므로 0.3도(=약 3스텝)면
 *  측정 잡음이 아니라 실제 탈조로 볼 수 있다.
 *  2.0도를 넘으면 기구 이상이다 — 계속해봐야 쓰레기 데이터라 중단한다. */
#define SCAN_RESYNC_DDEG      3     /* 0.3도 — 넘으면 재영점        */
#define SCAN_STALL_DDEG      20     /* 2.0도 — 넘으면 ERR_STALL     */

typedef enum {
    SC_IDLE = 0,
    SC_HOMING,       /* 엔코더 판독으로 절대 위치 확립 (구동 없음) */
    SC_MOVE_START,   /* 스캔 시작점으로 이동                        */
    SC_SWEEP,        /* 틸트 스윕 진행 중 — 이 구간의 점만 유효     */
    SC_LINE_END,     /* 줄 끝: 틸트 엔코더 대조 + 재영점            */
    SC_PAN_STEP,     /* 팬 1스텝 이동 + 팬 엔코더 검증              */
    SC_DONE          /* 완료 통지 후 IDLE                            */
} scan_state_t;

void scan_init(void);

/* CMD_HOME / CMD_SCAN_START / CMD_SCAN_STOP 진입점 (uart_rpi 디스패처가 호출) */
void scan_home(void);
void scan_start(const struct proto_scan_start *ss);
void scan_stop(void);

/* 메인루프에서 매 바퀴 호출. 상태 전이와 엔코더 판독이 전부 여기서 일어난다. */
void scan_process(void);

bool         scan_is_busy(void);
scan_state_t scan_get_state(void);

/* --- 라이다 연동 (App/lidar 가 호출) --------------------------------------
 *
 *  각도-거리 짝짓기는 **STM32 안에서 원자적으로 끝나야** 한다. 프레임이
 *  완성된 순간의 각도를 잡아 두고, 파싱이 끝난 뒤 그 각도와 함께 제출한다.
 *  RPi 로는 이미 짝지어진 (pan, tilt, d) 가 올라가므로 연관 부담이 없다.
 *
 *  ① 라이다 UART ISR 에서 프레임 마지막 바이트를 받은 즉시:
 *        scan_latch_angles(&pan, &tilt);
 *     (모터 카운터를 읽기만 하므로 ISR 에서 안전하다)
 *
 *  ② 메인루프에서 프레임 파싱이 끝나면 ①의 각도와 함께:
 *        scan_submit_sample(pan, tilt, d_mm, ...);
 *
 *  스윕 구간(SC_SWEEP)이 아닐 때 들어온 샘플은 버린다 — 줄 끝 정지나
 *  팬 이동 중의 점은 격자에 넣을 각도가 아니기 때문. */
void scan_latch_angles(int16_t *out_pan_ddeg, int16_t *out_tilt_ddeg);

void scan_submit_sample(int16_t pan_ddeg, int16_t tilt_ddeg,
                        uint16_t d_mm, uint16_t signal_strength,
                        uint32_t device_time_ms,
                        uint8_t dis_status, uint8_t range_precision);

#endif /* SCAN_H */
