#include <stdint.h>
#include <stdbool.h>

/* VEDA ADTS 기획서 및 Device 아키텍처 규격 준수 고정 크기 구조체 */
struct TurretAngle {
    int16_t theta;      /* 방위각 (스텝모터 기준 절대각, 0.1도 단위, 0 ~ 3600) */
    int16_t phi;        /* 고각 (서보모터 제어각, 0.1도 단위, -900 ~ 900) */
} __attribute__((packed));

/* * [MISRA C:2023 Rule 8.4 준수] 내부 도우미 함수는 static 선언으로 파일 스코프 격리
 * [CWE-398 준수] outTarget 매개변수 주소 변조 가드용 포인터 const 지정
 */
static bool validateAndPackAngle(const int16_t rawTheta, const int16_t rawPhi, struct TurretAngle* const outTarget) {
    bool isValid = false;

    /* 안전한 포인터 Null 검사 강제 */
    if (outTarget != (void*)0) {
        /* 하드웨어 물리 제어 안전 경계값 조건 실시간 검증 */
        if ((rawTheta >= 0) && (rawTheta <= 3600) && (rawPhi >= -900) && (rawPhi <= 900)) {
            outTarget->theta = rawTheta;
            outTarget->phi = rawPhi;
            isValid = true;
        }
    }
    return isValid;
}

int main(void) {
    struct TurretAngle trackingDevice = {0, 0};
    
    /* [CWE-398] 변조 방지 상수가 가드 */
    const int16_t visualTargetTheta = 1205; /* AI 카메라 추적 방위각 (120.5도) */
    const int16_t visualTargetPhi = 450;    /* 영상 기반 고각 평활화각 (45.0도) */

    /* * [MISRA C:2023 Rule 17.7 정면 돌파] 
     * 반환 값이 있는 함수의 결과는 무조건 평가식에 사용하거나 핸들링해야 합니다.
     * 아래와 같이 조건문 내부에서 상태를 명확히 평가하여 노란색 MISRA 경고를 소멸시킵니다.
     */
    if (validateAndPackAngle(visualTargetTheta, visualTargetPhi, &trackingDevice) == true) {
        /* 패킹 성공: 통합 데몬의 unlocked_ioctl 하드웨어 UART 송신 시퀀스 연동 */
    } else {
        /* 검증 실패: 타깃 조준 실패 예외 에러 처리 대응 */
    }

    return 0;
}
