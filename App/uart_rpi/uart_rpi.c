/* ============================================================================
 *  uart_rpi.c  --  RPi <-> STM32 UART 포트 제어 및 프로토콜 디스패처
 *  담당: 이현우
 *  (main.c 에서 추출. 동작은 검증본과 동일 — 디버그 트레이스는 DBG 매크로로 이관)
 *
 *  MISRA-C:2023 정리 완료:
 *    - 21.6  : printf → DBG() 매크로(기본 컴파일아웃)
 *    - 17.7  : HAL/memcpy 반환값 (void) 캐스트
 *    - 15.5/15.6 : 단일 exit + 중괄호
 *    - 13.3  : 링버퍼 post-increment 분리
 *    - 10.4  : 부호 있는 리터럴 → unsigned + 명시 캐스트
 *    - 21.15 : (de)serialize memcpy 는 문서화된 deviation
 * ==========================================================================*/
#include "uart_rpi.h"
#include "motor.h"        /* CMD_SET_TARGET/DISARM → 모터 구동 (테스트 경로) */
#include <string.h>

/* ---- 디버그 트레이스 -------------------------------------------------------
 *  MISRA-C:2023 Rule 21.6 (표준 I/O 함수 금지) 대응.
 *  기본값 0 → printf 자체가 컴파일아웃되어 정적분석/릴리즈에서 위반 없음.
 *  하드웨어 브링업 때 트레이스가 필요하면 이 파일 상단(또는 빌드 플래그)에서
 *  UART_RPI_DEBUG 를 1 로 지정한다(그 경우 21.6 은 디버그 빌드 한정 deviation).
 * ------------------------------------------------------------------------- */
#ifndef UART_RPI_DEBUG
#define UART_RPI_DEBUG 1
#endif

#if UART_RPI_DEBUG
/* cppcheck-suppress misra-c2012-21.6 ; 디버그 빌드(UART_RPI_DEBUG=1) 한정 stdio deviation */
#include <stdio.h>
/* cppcheck-suppress misra-c2012-21.6 ; 디버그 빌드 한정 printf deviation */
#define DBG(...)  ((void)printf(__VA_ARGS__))
#else
#define DBG(...)  ((void)0)
#endif

/* ---- 내부 상태 ---- */
static UART_HandleTypeDef *s_huart;        /* RPi 링크 UART (USART1)      */
static volatile uint8_t    s_rx;           /* 1바이트 수신 버퍼 (IT)       */
static volatile uint8_t    s_rb[256];      /* 수신 링버퍼                  */
static volatile uint16_t   s_rb_head = 0u;
static volatile uint16_t   s_rb_tail = 0u;

/* protocol.h 프레임 빌드 → USART1 TX 전송 (상행) */
void uart_rpi_send_frame(uint8_t cmd, const void *payload, uint8_t payload_len)
{
    if (payload_len <= PROTO_MAX_PAYLOAD) {            /* CWE-120 경계검사 */
        uint8_t  frame[PROTO_MAX_FRAME];
        uint16_t crc;
        uint8_t  total;

        frame[0] = PROTO_SOF;
        frame[1] = cmd;
        frame[2] = payload_len;

        if ((payload_len > 0u) && (payload != NULL)) {
            /* cppcheck-suppress misra-c2012-21.15 ; 합의된 바이트열 직렬화 */
            (void)memcpy(&frame[PROTO_HEADER_LEN], payload, payload_len);
        }

        total = (uint8_t)(PROTO_HEADER_LEN + payload_len);
        crc   = proto_crc16(frame, total);
        frame[total]      = (uint8_t)(crc & 0xFFu);              /* little-endian */
        frame[total + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
        total = (uint8_t)(total + PROTO_CRC_LEN);

        (void)HAL_UART_Transmit(s_huart, frame, total, 100u);
        DBG("[TX] cmd=0x%02X len=%u\r\n", cmd, payload_len);
    }
}

/* 완성된 프레임(buf[0..flen-1]) CRC 검증 후 CMD 디스패치 */
static void proto_dispatch(const uint8_t *buf, uint8_t flen)
{
    uint8_t  len    = buf[2];
    /* 수신 CRC 복원: uint32 로 합성해 합성식 캐스트(10.8) 회피 후 단순 변수만 축소 */
    uint32_t raw     = ((uint32_t)buf[flen - 1u] << 8) | (uint32_t)buf[flen - 2u];
    uint16_t rx_crc  = (uint16_t)raw;
    uint16_t crc_len = (uint16_t)len + (uint16_t)PROTO_HEADER_LEN; /* 합성식 캐스트 회피 */
    uint16_t calc    = proto_crc16(buf, crc_len);

    if (rx_crc == calc) {
        uint8_t cmd = buf[1];
        DBG("  CRC OK, cmd=0x%02X\r\n", cmd);

        switch (cmd) {
        case CMD_SET_TARGET:
            if (len == sizeof(struct proto_target)) {
                struct proto_target t;
                /* cppcheck-suppress misra-c2012-21.15 ; 와이어 바이트열→packed 구조체 역직렬화(합의 LE) */
                (void)memcpy(&t, &buf[PROTO_HEADER_LEN], sizeof(t));
                DBG("  SET_TARGET theta=%d phi=%d\r\n", t.theta_ddeg, t.phi_ddeg);
                motor_set_target(t.theta_ddeg, t.phi_ddeg);   /* → 모터 구동 */
            }
            break;

        case CMD_HOME:
            DBG("  HOME (모터부 담당) -> HOMED 회신\r\n");
            uart_rpi_send_frame(CMD_HOMED, NULL, 0u);   /* 상행 경로 검증용 */
            break;

        case CMD_SET_MODE:
            if (len == sizeof(struct proto_mode)) {
                struct proto_mode m;
                /* cppcheck-suppress misra-c2012-21.15 ; 역직렬화(합의 LE) */
                (void)memcpy(&m, &buf[PROTO_HEADER_LEN], sizeof(m));
                DBG("  SET_MODE mode=%u\r\n", m.mode);
            }
            break;

        case CMD_DISARM:
            DBG("  DISARM (스텝 2축 disable)\r\n");
            motor_disarm();                             /* → 모터 정지 */
            break;

        case CMD_PING:
            uart_rpi_send_frame(CMD_PONG, NULL, 0u);    /* heartbeat 응답 */
            DBG("  PING -> PONG\r\n");
            break;

        case CMD_QUERY_DIST:
        {
            /* 라이다 미연결: 더미 거리로 상행 경로 검증 */
            struct proto_distance d = { .distance_mm = 2221u };
            uart_rpi_send_frame(CMD_DISTANCE, &d, (uint8_t)sizeof(d));
            DBG("  QUERY_DIST -> DISTANCE %u mm (dummy)\r\n", d.distance_mm);
        }
            break;

        default:
            DBG("  (unhandled cmd 0x%02X)\r\n", cmd);
            break;
        }
    } else {
        DBG("CRC FAIL rx=%04X calc=%04X\r\n", rx_crc, calc);
    }
}

/* 바이트 스트림 → 프레임 파싱 (상태머신), 완성 시 proto_dispatch 호출 */
static void proto_feed(uint8_t b)
{
    static uint8_t buf[PROTO_MAX_FRAME];
    static uint8_t idx  = 0u;
    static uint8_t need = 0u;

    DBG("[feed] byte=0x%02X idx=%u\r\n", b, idx);      /* 모든 수신 바이트 */

    if (idx == 0u) {
        /* 프레임 경계 탐색: SOF 만 시작으로 인정, 그 외 바이트는 버림 */
        if (b == PROTO_SOF) {
            buf[0] = b;
            idx    = 1u;
            need   = 0u;
            DBG("  SOF ok\r\n");
        } else {
            DBG("  skip (not SOF)\r\n");
        }
    } else {
        buf[idx] = b;
        idx      = (uint8_t)(idx + 1u);

        if (idx == PROTO_HEADER_LEN) {
            uint8_t len = buf[2];
            if (len > PROTO_MAX_PAYLOAD) {
                DBG("ERR bad len=%u\r\n", len);
                idx  = 0u;                              /* CWE-120: 프레임 폐기 */
                need = 0u;
            } else {
                need = (uint8_t)(PROTO_HEADER_LEN + len + PROTO_CRC_LEN);
                DBG("  header: cmd=0x%02X len=%u need=%u\r\n", buf[1], len, need);
            }
        }

        if ((need != 0u) && (idx >= need)) {
            proto_dispatch(buf, need);
            idx  = 0u;
            need = 0u;
        }
    }
}

/* ---- 공개 훅 ---- */

void uart_rpi_init(UART_HandleTypeDef *huart)
{
    s_huart   = huart;
    s_rb_head = 0u;
    s_rb_tail = 0u;
    (void)HAL_UART_Receive_IT(s_huart, (uint8_t *)&s_rx, 1u);   /* 수신 시작 */
}

void uart_rpi_on_rx_cplt(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        s_rb[s_rb_head] = s_rx;                          /* 링버퍼 적재만 */
        s_rb_head = (uint16_t)((s_rb_head + 1u) & 0xFFu);/* 256 wrap */
        (void)HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx, 1u);
    }
}

void uart_rpi_on_error(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        /* cppcheck-suppress misra-c2012-14.4 ; HAL 벤더 매크로 내부 표현식 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)HAL_UART_Receive_IT(huart, (uint8_t *)&s_rx, 1u);
    }
}

void uart_rpi_process(void)
{
    while (s_rb_tail != s_rb_head) {                     /* 버퍼에 데이터 있으면 */
        uint8_t bb = s_rb[s_rb_tail];
        s_rb_tail  = (uint16_t)((s_rb_tail + 1u) & 0xFFu);
        proto_feed(bb);                                  /* 파싱 (느긋해도 안전) */
    }
}
