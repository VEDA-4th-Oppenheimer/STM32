cmake_minimum_required(VERSION 3.15)
project(VEDA_ADTS_HOST_ANALYSIS C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =====================================================================
# 정적 분석 (Cppcheck) — ENABLE_STATIC_ANALYSIS=ON 일 때만 활성
# =====================================================================
option(ENABLE_STATIC_ANALYSIS "Enable Cppcheck static analysis" OFF)

if(ENABLE_STATIC_ANALYSIS)
    find_program(CPPCHECK_PATH cppcheck)
    if(CPPCHECK_PATH)
        message(STATUS "🔥 [정적분석] cppcheck 장착: ${CPPCHECK_PATH}")
        set(CPPCHECK_ARGS
            "--enable=warning,style,performance,portability"
            "--suppressions-list=${CMAKE_SOURCE_DIR}/deamon/cppcheck_suppressions.txt"
            "--inline-suppr"
            "--inconclusive"
            "--error-exitcode=1"          # ← 지적사항 있으면 빌드 실패 = 진짜 게이트
        )
        set(CMAKE_C_CPPCHECK   "${CPPCHECK_PATH}" ${CPPCHECK_ARGS})
        set(CMAKE_CXX_CPPCHECK "${CPPCHECK_PATH}" ${CPPCHECK_ARGS})
    else()
        message(WARNING "cppcheck 미설치 — 정적분석 건너뜀 (brew install cppcheck)")
    endif()
endif()

# =====================================================================
# 호스트측 분석 대상 소스 자동 수집
#   - 펌웨어(src/USART1)는 arm-none-eabi 크로스 빌드라 제외
#     (펌웨어는 src/USART1 의 자체 CMake preset + compile_commands.json 로 분석)
# =====================================================================
file(GLOB_RECURSE HOST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)
list(FILTER HOST_SOURCES EXCLUDE REGEX "/src/USART1/")

if(NOT HOST_SOURCES)
    message(FATAL_ERROR "❌ 분석할 호스트 소스가 없습니다 (src/USART1 제외).")
endif()

# =====================================================================
# 각 소스를 독립 실행 파일로 등록
#   - 타겟명은 상대경로 기반으로 생성해 파일명 충돌(중복 타겟) 원천 차단
#     예) src/test3/test_c.c -> test3_test_c
# =====================================================================
foreach(SRC ${HOST_SOURCES})
    file(RELATIVE_PATH REL "${CMAKE_SOURCE_DIR}/src" "${SRC}")
    string(REGEX REPLACE "\\.[^.]*$" "" REL "${REL}")   # 확장자 제거
    string(REGEX REPLACE "[/\\.]" "_" TGT "${REL}")     # '/' '.' -> '_'
    add_executable(${TGT} "${SRC}")
    message(STATUS "🛠️  타겟: ${TGT}  <-  ${SRC}")
endforeach()
