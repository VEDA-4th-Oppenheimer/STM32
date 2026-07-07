#include <iostream>
#include <stdexcept>
#include <memory>  // Modern C++ 스마트 포인터 사용을 위해 필수 포함

// 드론 제어 클래스 정의
class DroneController {
public:
    void startTracking() const {
        std::cout << "Target tracking started successfully." << std::endl;
    }
};

void processDroneData(int mode) {
    // -----------------------------------------------------------------
    // [해결] CWE-401 메모리 누수 및 C++ 가이드라인 위반 원천 차단
    // -----------------------------------------------------------------
    // 레거시 스타일의 'new' 대신 현대적 표준인 'std::unique_ptr'(스마트 포인터)를 사용합니다.
    // 이렇게 하면 내부에서 예외(throw)가 터져 함수가 조기 종료되더라도, 
    // 스택이 파괴되면서 자동으로 메모리가 안전하게 해제(RAII 규격)되므로 누수가 불가능해집니다.
    std::unique_ptr<DroneController> controller = std::make_unique<DroneController>();

    if (mode == 1) {
        // 예외가 발생하더라도 스마트 포인터 가드가 작동하여 controller 자원이 자동 수거됩니다.
        throw std::runtime_error("Critical Hardware Error Context Interrupted!");
    }

    controller->startTracking();
    
    // -----------------------------------------------------------------
    // [해결] NULL 매크로 대신 nullptr 사용 강제 수칙 준수
    // -----------------------------------------------------------------
    // 형안정성이 깨지는 레거시 NULL 대신 Modern C++ 규격인 nullptr를 정조준합니다.
    int* modernPointer = nullptr; 

    if (modernPointer == nullptr) {
        std::cout << "Pointer is initialized safely." << std::endl;
    }
}

int main() {
    try {
        processDroneData(0); // 🟢 정상 검증 확인 모드로 구동 (예외 유발 X)
    } catch (const std::exception& e) {
        std::cerr << "Caught Exception: " << e.what() << std::endl;
    }
    return 0;
}
