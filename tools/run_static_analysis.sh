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
cd "$(dirname "$0")/.."          # tools/ -> repo 루트

SUPPRESS="tools/cppcheck_suppressions.txt"
BUILD_DIR="build/Debug"

echo "==> [1/2] compile_commands.json 재생성 (arm-none-eabi)"
cmake --preset Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null

echo "==> [2/2] cppcheck (우리 코드 Core/Src 대상)"
cppcheck \
  --project="${BUILD_DIR}/compile_commands.json" \
  --file-filter="*/App/*" \
  --file-filter="*/Core/Src/main.c" \
  -D__GNUC__=15 \
  --enable=warning,style,performance,portability \
  --inline-suppr \
  --suppressions-list="${SUPPRESS}" \
  --error-exitcode=1 \
  --template="{severity}|{file}:{line}| {message} ({id})"

echo "==> 정적분석 통과 ✅"
