/* ============================================================================
 *  hallEffectSensor.c  --  MT6701 자기 각도 엔코더 I2C 판독 구현
 *  최초 작성: 2026-07-23
 *  담당: 강유근 (원 구현) / 이현우 (NULL 가드 교정)
 *  계약과 배선은 hallEffectSensor.h 상단 참조.
 * ==========================================================================*/

#include "hallEffectSensor.h"
/* 9클럭 복구가 SCL/SDA 를 GPIO 로 직접 흔들어야 해서 CubeMX 핀 라벨이 필요하다.
 * 핀을 옮기면 .ioc 재생성으로 main.h 가 바뀌고 여기는 자동으로 따라온다. */
#include "main.h"
#include <stddef.h>
#include <stdbool.h>

/* MT6701 각도 레지스터 비트 배치 (데이터시트):
 *   0x03 : Angle[13:6]           -> rx_buf[0]
 *   0x04 : Angle[5:0] << 2       -> rx_buf[1] 의 상위 6비트
 * 따라서 raw = (rx[0] << 6) | (rx[1] >> 2) 로 14비트를 복원한다.
 * 결과는 항상 0 ~ 16383 안에 들어가므로 별도 범위 검사가 필요 없다. */
#define MT6701_ANGLE_HI_SHIFT   6U
#define MT6701_ANGLE_LO_SHIFT   2U
#define MT6701_COUNTS           16384.0f   /* 2^14 = 한 바퀴 */

HAL_StatusTypeDef Encoder_Read(I2C_HandleTypeDef *hi2c, Encoder_t *encoder_data)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    /* 주의: 여기는 원래 `||` 였다. 그러면 핸들만 유효하고 결과 포인터가 NULL 인
     *   경우에도 통과해 encoder_data->raw_angle 에서 NULL 역참조로 죽는다.
     *   가드가 정반대로 동작하고 있었다. 두 인자가 **모두** 유효해야 한다. */
    if ((hi2c != NULL) && (encoder_data != NULL)) {
        uint8_t rx_buf[2]  = {0};
        uint8_t reg_addr   = (uint8_t)REG_ANGLE_14B;

        /* 0x03 레지스터부터 2바이트.
         * 핸들은 축에 따라 다르다 — Pan=I2C3(PA8/PC9), Tilt=I2C1(PB8/PB9).
         * (원 주석은 "I2C1: PA8/PC9" 로 둘을 뒤섞어 적고 있었다) */
        /* 주의: 원래는 HAL_I2C_Mem_Read 였다. 그건 레지스터 주소를 쓴 뒤
         *   **repeated start** 로 읽기로 전환하는데, 실기에서 그 repeated
         *   start 가 400kHz 마진을 못 버티고 NACK(err=0x04) 났다. 벤치의
         *   방식 탐색 결과가 근거다(2026-08-05):
         *
         *     A Mem_Read(400k)      실패 NACK
         *     B Transmit+Receive    OK  raw=3531
         *     C Receive만           "OK" 지만 raw=0  <- 포인터 없이 읽어 무의미
         *     D Mem_Read(100k)      OK  raw=3531
         *
         *   B 와 D 가 같은 값을 냈다 = 3531 이 진짜 각도. 원인은 방식이
         *   아니라 400kHz 에서의 repeated start 타이밍 마진 부족이고,
         *   B(중간에 STOP) 와 D(속도 하향) 둘 다 회피책이다. 둘 다 적용해
         *   마진을 최대로 둔다 — 엔코더는 홈 확립에 쓰이고 홈이 틀리면
         *   좌표계 전체가 틀어지므로 "겨우 되는" 상태로 두면 안 된다.
         *
         *   주의: 순서 의존: Transmit 이 성공해야만 Receive 가 의미 있다. */
        status = HAL_I2C_Master_Transmit(hi2c, MT6701_ADDR, &reg_addr, 1u,
                                         I2C_TIMEOUT);
        if (status == HAL_OK) {
            status = HAL_I2C_Master_Receive(hi2c, MT6701_ADDR, rx_buf, 2u,
                                            I2C_TIMEOUT);
        }

        if (status == HAL_OK) {
            const uint16_t raw =
                (uint16_t)(((uint16_t)rx_buf[0] << MT6701_ANGLE_HI_SHIFT) |
                           ((uint16_t)rx_buf[1] >> MT6701_ANGLE_LO_SHIFT));

            encoder_data->raw_angle = raw;
            encoder_data->degree    = (float)raw * (360.0f / MT6701_COUNTS);
        }
        /* 실패 시 encoder_data 는 건드리지 않는다 — 호출자가 status 를 보고
         * 판단해야 한다(헤더의 경고 참조). */
    }

    return status;
}

/* 축에 물린 SCL/SDA 핀. 9클럭 복구에서 GPIO 로 직접 흔들어야 한다.
 * Core/Inc/main.h 의 CubeMX 라벨을 그대로 쓴다 — 핀을 옮기면 .ioc 재생성으로
 * 그쪽이 바뀌고 여기는 자동으로 따라온다. */
struct i2c_pins {
    GPIO_TypeDef *scl_port; uint16_t scl_pin;
    GPIO_TypeDef *sda_port; uint16_t sda_pin;
};

static bool encoder_pins_of(const I2C_HandleTypeDef *hi2c, struct i2c_pins *out)
{
    bool ok = true;

    if (hi2c->Instance == I2C1) {          /* 틸트 */
        out->scl_port = TILT_ENCODER_SCL_GPIO_Port;
        out->scl_pin  = TILT_ENCODER_SCL_Pin;
        out->sda_port = TILT_ENCODER_SDA_GPIO_Port;
        out->sda_pin  = TILT_ENCODER_SDA_Pin;
    } else if (hi2c->Instance == I2C3) {   /* 팬 */
        out->scl_port = PAN_ENCODER_SCL_GPIO_Port;
        out->scl_pin  = PAN_ENCODER_SCL_Pin;
        out->sda_port = PAN_ENCODER_SDA_GPIO_Port;
        out->sda_pin  = PAN_ENCODER_SDA_Pin;
    } else {
        ok = false;
    }
    return ok;
}

/* SCL 을 최대 9번 토글해 슬레이브가 붙잡고 있는 SDA 를 놓게 한다.
 *
 * 핵심: 왜 9번인가: 슬레이브가 바이트 전송 중간에 갇히면 남은 비트 수만큼 클럭을
 *   더 받아야 그 바이트를 끝내고 SDA 를 놓는다. 최악이 8비트 + ACK = 9 다.
 *   SDA 가 올라오면 즉시 중단한다.
 *
 * 끝나고 STOP 조건(SCL high 동안 SDA low->high)을 만들어 슬레이브를 idle 로
 * 되돌린다. 이게 없으면 슬레이브는 아직 전송 중이라 여긴다. */
static void encoder_clock_out(const struct i2c_pins *p)
{
    GPIO_InitTypeDef g = {0};

    g.Mode  = GPIO_MODE_OUTPUT_OD;    /* 오픈드레인 — 풀업이 high 를 만든다 */
    /* cppcheck-suppress misra-c2012-7.3 ; 애드온 오탐 (아래 근거)
     *
     * 주의: 우리 코드에는 소문자 l 접미사가 없다. cppcheck misra.py 의 7.3 정규식이
     *      ^(0[xX])?[0-9a-fA-FpP.]+[Uu]*l+[Uu]*$
     *   인데 `0x` 접두가 **선택**이고 문자 집합에 P 가 들어 있어서, 식별자
     *   `Pull` 이  P(=수) + u + ll(=접미사)  로 해석된다. 즉 GPIO_InitTypeDef 의
     *   .Pull 멤버를 쓰는 모든 코드가 걸린다.
     *   CubeMX 생성 코드가 안 걸리는 건 Core/ 를 통째로 억제하고 있어서다(§14).
     *   억제 대신 필드명을 피할 방법이 없으므로 여기서 근거를 달고 넘어간다. */
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = p->scl_pin;  HAL_GPIO_Init(p->scl_port, &g);
    g.Pin = p->sda_pin;  HAL_GPIO_Init(p->sda_port, &g);

    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_SET);   /* SDA 놓기 */

    for (uint32_t i = 0u; i < 9u; i++) {
        if (HAL_GPIO_ReadPin(p->sda_port, p->sda_pin) == GPIO_PIN_SET) {
            break;                       /* 슬레이브가 놓았다 */
        }
        HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1u);                   /* 100kHz 보다 훨씬 느리게 — 안전측 */
        HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_SET);
        HAL_Delay(1u);
    }

    /* STOP: SCL high 인 동안 SDA 를 low -> high */
    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_RESET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(p->scl_port, p->scl_pin, GPIO_PIN_SET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(p->sda_port, p->sda_pin, GPIO_PIN_SET);
    HAL_Delay(1u);
}

/* 헤더의 설명 참조.
 *
 * 주의: **DeInit + Init 만으로는 부족했다** (2026-08-13 실기). MspDeInit 이 하는
 *   것은 `__HAL_RCC_I2Cx_CLK_DISABLE()` + GPIO 해제뿐인데, **클럭을 끄는 것은
 *   페리페럴을 리셋하는 것이 아니다.** 내부 상태머신과 레지스터가 그대로 남아
 *   있다가 클럭을 다시 켜면 그 상태로 살아나므로 BUSY 래치가 안 풀린다.
 *   NRST 로는 복구되는데 이 함수로는 안 되던 이유가 이것이다.
 *
 *   그래서 RCC 강제 리셋을 넣는다 — NRST 가 하는 것을 그 페리페럴에만 한다.
 *
 * 순서가 중요하다:
 *   ① 9클럭 + STOP  — 버스(슬레이브)를 먼저 풀어준다. 페리페럴을 되살려도
 *                     버스가 잡혀 있으면 첫 START 에서 곧바로 다시 막힌다.
 *   ② RCC 강제 리셋 — 페리페럴 내부 상태를 진짜로 지운다.
 *   ③ Init          — CubeMX 설정(100kHz 등)으로 재구성. hi2c->Init 이
 *                     구조체에 남아 있어 인자를 따로 들 필요가 없다. */
HAL_StatusTypeDef Encoder_BusRecover(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    struct i2c_pins pins;

    if ((hi2c != NULL) && encoder_pins_of(hi2c, &pins)) {
        (void)HAL_I2C_DeInit(hi2c);      /* 클럭 끄고 GPIO 를 우리 손으로 */
        encoder_clock_out(&pins);        /* ① 버스 풀기 */

        if (hi2c->Instance == I2C1) {    /* ② 페리페럴 진짜 리셋 */
            __HAL_RCC_I2C1_FORCE_RESET();
            __HAL_RCC_I2C1_RELEASE_RESET();
        } else {
            __HAL_RCC_I2C3_FORCE_RESET();
            __HAL_RCC_I2C3_RELEASE_RESET();
        }
        status = HAL_I2C_Init(hi2c);     /* ③ 재구성 */
    }
    return status;
}
