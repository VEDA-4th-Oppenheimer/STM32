#include <string.h>
#include <stdio.h>
#include <stdint.h>

// 전역 진입 함수 (링커 에러 방지)
int main(void) {
    // -----------------------------------------------------------------
    // 위반 1: CWE-120 (Buffer Overflow) 정적 확정 시나리오
    // -----------------------------------------------------------------
    char small_buffer[8] = {0};
    
    // Cppcheck 엔진이 정적으로 크기 계산이 가능한 명백한 오버플로우 데이터
    const char* dangerous_input = "VEDA_ANTI_DRONE_TARGETING_SYSTEM_OVERFLOW_TEST"; 
    
    // 복사 대상 버퍼(8바이트)보다 입력 문자열이 훨씬 크므로 100% 버퍼 오버플로우 발생
    // [CWE-120 보안위반] 및 [MISRA C 위반] 동시 검출 구간
    strcpy(small_buffer, dangerous_input); 


    // -----------------------------------------------------------------
    // 위반 2: MISRA C:2012 Rule 11.4 (Pointer Type Conversion)
    // -----------------------------------------------------------------
    uint32_t raw_address = 0x40020000; // STM32 GPIO 레지스터 가상 주소
    uint8_t* invalid_pointer;

    // 하드웨어 레지스터 제어를 위해 정수를 포인터로 강제 캐스팅하는 행위는
    // MISRA C 규칙상 엄격히 경고 대상입니다. (안전한 정밀 제어 규격 위반)
    invalid_pointer = (uint8_t*)raw_address; 


    // 정적 분석기 최적화로 인한 코드 생략 방지용 출력
    printf("Buffer: %s, Reg: %p\n", small_buffer, (void*)invalid_pointer);
    
    return 0;
}
