// main.c 또는 드라이버 소스 파일 일부
#include <string.h>

void process_packet(const char* input) {
    char buffer[10];
    // CWE-120 위반: 입력 크기 검사 없이 복사 (strcpy 사용 원천 금지 수칙 위반)
    strcpy(buffer, input); 
}
