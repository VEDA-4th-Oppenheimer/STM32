#include <stdint.h>
#include <stdbool.h>

/* VEDA ADTS Device 아키텍처 규격 준수 고정 크기 구조체 정의 */
struct TurretAngle {
    int16_t theta;      /* 방위각 (스텝모터 기준 절대각, 0.1도 단위, 0 ~ 3600) */
    int16_t phi;        /* 고각 (서보모터 제어각, 0.1도 단위, -900 ~ 900) */
} __attribute__((packed));

/*
 * [MISRA C:2023 Rule 8.4 준수] 
 * 외부 헤더에 노출되지 않는 내부 헬퍼 함수는 'static'을 명시하여 
 * Internal Linkage를 강제해야만 전역 기출 오염 경고(style)를 피할 수 있습니다.
 * * [CWE-398 / 품질 가이드 준수]
 * 전달받은 주소 자체의 변조를 막기 위해 포인터에 'const' 가드 가동.
 */
static bool validateAndPackAngle(int16_t rawTheta, int16_t rawPhi, struct TurretAngle* const outTarget) {
    bool isValid = false;

    /* 안전한 포인터 Null 검사 강제 */
    if (outTarget != (void*)0) {
        /* * 방위각 하드웨어 리미트 범위(0.0도~360.0도) 및 
         * 고각 기하학적 범위(-90.0도~90.0도)에 대한 하드웨어 방어적 경계값 검증 수행 
         */
        if ((rawTheta >= 0) && (rawTheta <= 3600) && (rawPhi >= -900) && (rawPhi <= 900)) {
            outTarget->theta = rawTheta;
            outTarget->phi = rawPhi;
            isValid = true;
        }
    }

    return isValid;
}

/* 시스템 메인 진입점 (링커의 undefined reference to 'main' 해결) */
int main(void) {
    struct TurretAngle trackingDevice = {0, 0};
    
    /* [CWE-398] 컴파일러 최적화 및 값 변조 방지를 위해 로컬 입력 상수에 const 적극 반영 */
    const int16_t visualTargetTheta = 1205; /* AI 카메라 추적 데이터 조대 회전각 (120.5도) */
    const int16_t visualTargetPhi = 450;    /* 영상 기반 고각 평활화 데이터 (45.0도) */

    /* * [MISRA C:2023 Rule 17.7 정면 돌파] 
     * void가 아닌 반환 값을 가진 함수의 리턴 데이터는 절대로 무시하거나 그냥 버리면 안 됩니다.
     * 아래와 같이 조건문(if)의 평가식 내부에서 명확하게 상태를 검증하고 사용해야만 
     * 노란색 MISRA 경고가 완전히 소멸합니다.
     */
    if (validateAndPackAngle(visualTargetTheta, visualTargetPhi, &trackingDevice) == true) {
        /* 패킹 성공 시: 향후 엣지 데몬의 unlocked_ioctl 하드웨어 UART 송신 루틴 연동 영역 */
    } else {
        /* 검증 실패 시: 타깃 조준 실패 예외 에러 핸들러 대응 영역 */
    }

    return 0;
}
