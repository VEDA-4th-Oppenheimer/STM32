/* ============================================================================
 *  step_motor.c
 *  Pan/Tilt 2축 스텝 모터 제어 구현부
 * ==========================================================================*/
#include "step_motor.h"
#include "uart_rpi.h" // 통신 피드백(CMD_HOMED, CMD_SCAN_DONE 등)용
#include <stdio.h>    // printf 사용을 위해 추가

/* 외부 I2C 핸들 (hallEffectSensor.c 연동용, main.c에 선언된 핸들을 사용한다고 가정) */
extern I2C_HandleTypeDef hi2c3; // PAN용 I2C (PA8, PC9)
extern I2C_HandleTypeDef hi2c1; // TILT용 I2C (PB8, PB9)

/* 전역 제어 구조체 인스턴스 및 포인터 할당 */
static motor_ctrl_t motor_ctrl_instance = {0};
motor_ctrl_t *motor_ctrl = &motor_ctrl_instance;

/* --------------------------------------------------------------------------
 *  초기화 및 안전 제어 API
 * --------------------------------------------------------------------------*/
void step_motor_init(void)
{
    // 모터 드라이버 활성화 (LOW Active 가정, 하드웨어에 따라 변경)
    HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_RESET);

    motor_ctrl->state = MOTOR_IDLE;
    motor_ctrl->is_homed = 0;
}

void motor_disarm(void) // CMD_DISARM 시 즉시 중지 (uart_rpi.c에서 호출)
{
    // 모터 드라이버 비활성화 (전류 차단)
    HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);

    motor_ctrl->state = MOTOR_IDLE;
}

void step_motor_stop(void)
{
    motor_ctrl->state = MOTOR_IDLE;
}

/* --------------------------------------------------------------------------
 *  영점 설정 (Homing)
 *  - I2C 엔코더의 절대각도를 읽어 현재 위치를 펄스로 환산 후, 목표 위치를 0으로 설정
 * --------------------------------------------------------------------------*/
void step_motor_home(void)
{
    Encoder_t pan_enc;
    float pan_deg = 0.0f;

    // Pan 축 엔코더 절대각도 판독
    if (Encoder_Read(&hi2c3, &pan_enc) == HAL_OK) {
        pan_deg = pan_enc.degree - PAN_ZERO_OFFSET_DEG;
        motor_ctrl->current_pan_pulse = (int32_t)(pan_deg / DEG_PER_PULSE);
    }

    // Tilt는 임시로 0으로 설정
    motor_ctrl->current_tilt_pulse = 0;

    motor_ctrl->target_pan_pulse = 0;
    motor_ctrl->target_tilt_pulse = 0;
    motor_ctrl->state = MOTOR_HOMING;
}

/* --------------------------------------------------------------------------
 *  스캔 시작 (Protocol Command)
 * --------------------------------------------------------------------------*/
void step_motor_scan_start(const struct proto_scan_start *ss)
{
    if (!motor_ctrl->is_homed) {
        struct proto_err err = { .code = ERR_NOT_HOMED };
        uart_rpi_send_frame(CMD_ERROR, &err, sizeof(err));
        return;
    }

    // 단위 변환: 0.1도(ddeg) -> Pulse
    motor_ctrl->scan_pan_start_pulse  = DDEG_TO_PULSE(ss->pan_start_ddeg);
    motor_ctrl->scan_pan_end_pulse    = DDEG_TO_PULSE(ss->pan_end_ddeg);
    motor_ctrl->scan_tilt_start_pulse = DDEG_TO_PULSE(ss->tilt_start_ddeg);
    motor_ctrl->scan_tilt_end_pulse   = DDEG_TO_PULSE(ss->tilt_end_ddeg);
    motor_ctrl->scan_pan_step_pulse   = DDEG_TO_PULSE(ss->step_ddeg);

    // 스캔 시작 지점으로 우선 이동
    motor_ctrl->target_pan_pulse  = motor_ctrl->scan_pan_start_pulse;
    motor_ctrl->target_tilt_pulse = motor_ctrl->scan_tilt_start_pulse;

    motor_ctrl->state = MOTOR_SCAN_MOVING_TO_START;
}

/* --------------------------------------------------------------------------
 *  Pan 펄스 스텝 제어 (내부 함수)
 * --------------------------------------------------------------------------*/
static inline void step_pan(void)
{
    // 1. 드라이버가 꺼져있을까봐 무조건 켜기 (기존 PWM 코드와 동일하게 RESET)
    HAL_GPIO_WritePin(PAN_EN_GPIO_Port, PAN_EN_Pin, GPIO_PIN_RESET);

    // 2. 방향 설정 (만약 반대로 돌면 SET/RESET 위치만 바꿔주세요)
    if (motor_ctrl->current_pan_pulse < motor_ctrl->target_pan_pulse) {
        HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, GPIO_PIN_SET);
        motor_ctrl->current_pan_pulse++;
    } else {
        HAL_GPIO_WritePin(PAN_DIR_GPIO_Port, PAN_DIR_Pin, GPIO_PIN_RESET);
        motor_ctrl->current_pan_pulse--;
    }

    // 3. 펄스 발생 (PWM 듀티를 흉내 내어 드라이버가 확실히 인식하게 만들기)
    HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_SET);

    // 딜레이를 대폭 증가! (약 0.3ms 대기. CPU에 무리가 안 가면서 드라이버 인식엔 충분한 시간)
    for (volatile int i = 0; i < 5000; i++);

    HAL_GPIO_WritePin(PAN_STEP_GPIO_Port, PAN_STEP_Pin, GPIO_PIN_RESET);
}

/* --------------------------------------------------------------------------
 *  Tilt 펄스 스텝 제어 (내부 함수)
 * --------------------------------------------------------------------------*/
static inline void step_tilt(void)
{
    // 1. 드라이버 무조건 켜기
    HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_RESET);

    // 2. 방향 설정
    if (motor_ctrl->current_tilt_pulse < motor_ctrl->target_tilt_pulse) {
        HAL_GPIO_WritePin(TILT_DIR_GPIO_Port, TILT_DIR_Pin, GPIO_PIN_SET);
        motor_ctrl->current_tilt_pulse++;
    } else {
        HAL_GPIO_WritePin(TILT_DIR_GPIO_Port, TILT_DIR_Pin, GPIO_PIN_RESET);
        motor_ctrl->current_tilt_pulse--;
    }

    // 3. 펄스 발생
    HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 5000; i++);

    HAL_GPIO_WritePin(TILT_STEP_GPIO_Port, TILT_STEP_Pin, GPIO_PIN_RESET);
}

/* --------------------------------------------------------------------------
 *  Pan 타이머 인터럽트 핸들러 (400Hz)
 * --------------------------------------------------------------------------*/
void step_motor_pan_isr(void)
{
    if (motor_ctrl->state == MOTOR_IDLE || motor_ctrl->state == MOTOR_SCAN_TILT_SWEEP) {
        return; // 현재 Pan 작동 불필요
    }

    if (motor_ctrl->current_pan_pulse != motor_ctrl->target_pan_pulse) {
        step_pan();
    } else {
        // 도착 완료 처리 로직
        if (motor_ctrl->state == MOTOR_HOMING) {
            if (motor_ctrl->current_tilt_pulse == motor_ctrl->target_tilt_pulse) {
                motor_ctrl->is_homed = 1;
                motor_ctrl->state = MOTOR_IDLE;
                uart_rpi_send_frame(CMD_HOMED, NULL, 0);

                /* ==[디버그]== */
                Encoder_t pan_enc;
                float pan_real_deg = 0.0f;
                if (Encoder_Read(&hi2c3, &pan_enc) == HAL_OK) {
                    pan_real_deg = pan_enc.degree - PAN_ZERO_OFFSET_DEG;
                }
                printf("[DEBUG] Homing Done -> Pan Encoder: %.2f deg (Pulse: %ld)\r\n",
                       pan_real_deg, motor_ctrl->current_pan_pulse);
                /* ==[디버그]== */

            }
        }
        else if (motor_ctrl->state == MOTOR_SCAN_MOVING_TO_START) {
            if (motor_ctrl->current_tilt_pulse == motor_ctrl->target_tilt_pulse) {
                /* ==[디버그]== */
                Encoder_t pan_enc;
                float pan_real_deg = 0.0f;
                if (Encoder_Read(&hi2c3, &pan_enc) == HAL_OK) {
                    pan_real_deg = pan_enc.degree - PAN_ZERO_OFFSET_DEG;
                }
                printf("[DEBUG] Reached Scan Start -> Pan Encoder: %.2f deg\r\n", pan_real_deg);
                /* ==[디버그]== */

                // 시작점 도달, 첫 틸트 스윕 개시
                motor_ctrl->state = MOTOR_SCAN_TILT_SWEEP;
                motor_ctrl->target_tilt_pulse = motor_ctrl->scan_tilt_end_pulse;
            }
        }
        else if (motor_ctrl->state == MOTOR_SCAN_PAN_STEP) {
            // Pan step 이동 완료
            if (motor_ctrl->current_pan_pulse >= motor_ctrl->scan_pan_end_pulse) {

                /* ==[디버그]== */
                printf("[DEBUG] Scan Finished! Returning to Home...\r\n");
                /* ==[디버그]== */

                // 전체 스캔 완료 -> 영점 회귀
                motor_ctrl->state = MOTOR_IDLE;
                uart_rpi_send_scan_done();
                step_motor_home();
            } else {

                /* ==[디버그]== */
                Encoder_t pan_enc;
                float pan_real_deg = 0.0f;
                if (Encoder_Read(&hi2c3, &pan_enc) == HAL_OK) {
                    pan_real_deg = pan_enc.degree - PAN_ZERO_OFFSET_DEG;
                }
                printf("[DEBUG] Pan Step Done -> Pan Encoder: %.2f deg\r\n", pan_real_deg);
                /* ==[디버그]== */

                // 다음 틸트 스윕 개시 (지그재그 반전)
                motor_ctrl->state = MOTOR_SCAN_TILT_SWEEP;
                if (motor_ctrl->target_tilt_pulse == motor_ctrl->scan_tilt_start_pulse) {
                    motor_ctrl->target_tilt_pulse = motor_ctrl->scan_tilt_end_pulse;
                } else {
                    motor_ctrl->target_tilt_pulse = motor_ctrl->scan_tilt_start_pulse;
                }
            }
        }
    }
}

/* --------------------------------------------------------------------------
 *  Tilt 타이머 인터럽트 핸들러 (800Hz)
 * --------------------------------------------------------------------------*/
void step_motor_tilt_isr(void)
{
    if (motor_ctrl->state == MOTOR_IDLE || motor_ctrl->state == MOTOR_SCAN_PAN_STEP) {
        return; // 현재 Tilt 작동 불필요
    }

    if (motor_ctrl->current_tilt_pulse != motor_ctrl->target_tilt_pulse) {
        step_tilt();
    } else {
        // 도착 완료 처리 로직
        if (motor_ctrl->state == MOTOR_HOMING) {
            if (motor_ctrl->current_pan_pulse == motor_ctrl->target_pan_pulse) {
                motor_ctrl->is_homed = 1;
                motor_ctrl->state = MOTOR_IDLE;
                uart_rpi_send_frame(CMD_HOMED, NULL, 0);
            }
        }
        else if (motor_ctrl->state == MOTOR_SCAN_MOVING_TO_START) {
            if (motor_ctrl->current_pan_pulse == motor_ctrl->target_pan_pulse) {
                motor_ctrl->state = MOTOR_SCAN_TILT_SWEEP;
                motor_ctrl->target_tilt_pulse = motor_ctrl->scan_tilt_end_pulse;
            }
        }
        else if (motor_ctrl->state == MOTOR_SCAN_TILT_SWEEP) {

            /* ==[디버그]== */
            printf("[DEBUG] Tilt Sweep Line Done -> Current Tilt (Pulse-based): %.2f deg\r\n",
                   (float)motor_ctrl->current_tilt_pulse * DEG_PER_PULSE);
            /* ==[디버그]== */

            // 틸트 1줄 스윕 완료 -> Pan을 설정된 간격만큼 스텝 이동
            motor_ctrl->state = MOTOR_SCAN_PAN_STEP;
            motor_ctrl->target_pan_pulse += motor_ctrl->scan_pan_step_pulse;
        }
    }
}