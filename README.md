A.D.T.S (Anti-Drone Tracking & Targeting System)
Team Oppenheimer | VEDA 4기 최종 프로젝트

📝 프로젝트 개요
본 프로젝트는 소형 드론의 불규칙한 고속 비행을 실시간으로 추적하고 정밀 타겟팅하기 위한 안티드론 통합 솔루션입니다. 고성능 영상 스트리밍 분석과 커널 레벨의 하드웨어 제어를 결합하여, 실시간 동기화 성능과 시스템 안정성을 최우선으로 설계되었습니다.

🏗️ 시스템 아키텍처
카메라의 영상 데이터를 받아 비전 알고리즘으로 타겟을 추적하고, 커널 드라이버를 통해 터렛(Turret)의 방위각과 고각을 제어합니다.

🚀 주요 기능
고속 데이터 파싱: 커널-사용자 공간 간 컨텍스트 스위칭 최소화를 위한 unlocked_ioctl 드라이버 설계.
좌표 정밀 제어: 칼만 필터(Kalman Filter) 기반의 타겟 경로 예측 및 평활화 알고리즘 적용.
보안 강화: TLS 기반 암호화 통신 및 정적 분석 가드레일 적용.
연속성 보장: 24시간 장기 구동을 위한 메모리 누수 관리(CWE-401 차단).

🛡️ 품질 관리 및 가드레일 (QA)
본 프로젝트는 코드 무결성을 위해 2-Track 정적 분석 파이프라인을 운영합니다.
Track 1 (C): Cppcheck & MISRA C:2023 적용 (보안 위반 CWE-120 방지)
Track 2 (C++): Clang-Tidy 적용 (Modern C++ 가이드라인 준수)
CI/CD: GitHub Actions를 통해 PR 병합 전 정적 분석 검사를 강제합니다. (분석 실패 시 Merge 차단)

📦 개발 가이드라인
Coding Convention: PascalCase(클래스), camelCase(함수/변수) 명명 규칙 등 Coding Convention을 준수합니다.
Commit Policy: 기능 단위로 커밋하며, main 브랜치는 정적 분석을 통과한 코드만 머지 가능합니다.
Static Analysis: 로컬 환경에서 ./build_run.sh clean을 통해 반드시 사전 검사를 수행하십시오.

본 프로젝트는 VEDA 프로젝트 정적 분석 및 예외 관리 가이드라인 및 Coding Convention을 준수합니다.
