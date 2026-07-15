#!/usr/bin/env bash
# =====================================================================
# STM32 펌웨어 정적분석 (cppcheck, compile_commands.json 기반)
#   - CubeMX가 생성한 CMake preset(Debug)로 compile_commands.json 재생성
#   - CMSIS "#error Unknown compiler" 방지: -D__GNUC__ 정의
#   - 벤더/자동생성 제외: tools/cppcheck_suppressions.txt
#   - 지적사항 있으면 exit 1 (CI 게이트)
# 위치: tools/ (스크립트는 repo 루트에서 동작)
# 사용법:  ./tools/run_static_analysis.sh   (또는 bash tools/run_static_analysis.sh)
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")/.."          # tools/ -> repo 루트로 이동

SUPPRESS="tools/cppcheck_suppressions.txt"
BUILD_DIR="build/Debug"
MISRA_JSON="tools/misra.json"

echo "==> [1/3] MISRA 애드온 설정 파일 동적 생성"
# tools/misra_rules.txt의 규칙 텍스트 정보를 참조하도록 json 자동 빌드
cat <<EOF > "${MISRA_JSON}"
{
    "script": "misra.py",
    "args": [
        "--rule-texts=tools/misra_rules.txt"
    ]
}
EOF

echo "==> [2/3] compile_commands.json 재생성 (arm-none-eabi)"
cmake --preset Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null

echo "==> [3/3] cppcheck (프로젝트 전체 검사 및 CWE/MISRA 탐지)"
cppcheck \
  --project="${BUILD_DIR}/compile_commands.json" \
  -i Drivers \
  -D__GNUC__=15 \
  --enable=warning,style,performance,portability \
  --inline-suppr \
  --addon="${MISRA_JSON}" \
  --suppressions-list="${SUPPRESS}" \
  --error-exitcode=1 \
  --template="{severity}|{file}:{line}| {message} ({id}) [CWE-{cwe}]"

echo "==> 정적분석 통과 ✅"
