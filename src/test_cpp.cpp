#include <iostream>
#include <stdexcept>

// 드론 제어 클래스 가상 정의
class DroneController {
public:
    void startTracking() {
        std::cout << "Target tracking started..." << std::endl;
    }
};

void processDroneData(int mode) {
    // -----------------------------------------------------------------
    // 위반 1: CWE-401 (Memory Leak) / 자원 누수 보안 취약점
    // -----------------------------------------------------------------
    // 힙(Heap) 영역에 메모리를 동적 할당했으나, 하단 예외 처리 로직에 의해
    // delete가 호출되지 못하고 함수가 종료되어 자원이 영구 누수되는 취약점입니다.
    DroneController* controller = new DroneController();

    if (mode == 1) {
        // 예외가 터지면서 함수를 즉시 탈출하게 됨
        // 이로 인해 하단의 'delete' 로직을 건너뛰어 동적 객체가 메모리에 그대로 박제됩니다.
        throw std::runtime_error("Critical Hardware Error Context Interrupted!");
    }

    controller->startTracking();
    
    // 정상 흐름에서만 해제됨 (중요 결함)
    delete controller; 


    // -----------------------------------------------------------------
    // 위반 2: Modern C++ readability (NULL 대신 nullptr 사용 강제 수칙)
    // -----------------------------------------------------------------
    // Modern C++ 코딩 컨벤션 및 Clang-Tidy 가이드라인에 따라
    // 전통적인 레거시 'NULL' 매크로 대신 명확한 형안정성을 지닌 'nullptr'을 써야 합니다.
    int* legacy_pointer = NULL; 

    if (legacy_pointer == NULL) {
        std::cout << "Pointer is uninitialized." << std::endl;
    }
}

int main() {
    try {
        processDroneData(1); // 의도적으로 예외 모드 진입시켜 누수 유발
    } catch (const std::exception& e) {
        std::cerr << "Caught Exception: " << e.what() << std::endl;
    }
    return 0;
}
