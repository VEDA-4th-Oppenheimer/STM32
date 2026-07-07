# 1. CMake 필수 기본 설정
cmake_minimum_required(VERSION 3.15)
project(VEDA_Static_Analysis_Test LANGUAGES C CXX)

# 2. 정적 분석 활성화 옵션 (기본값 ON)
option(ENABLE_STATIC_ANALYSIS "Enable Cppcheck(C) and Clang-Tidy(C++)" ON)

if(ENABLE_STATIC_ANALYSIS)
    message(STATUS "🔥 [2-Track] 전역 정적 분석 가드레일이 활성화되었습니다!")

    # =================================================================
    # TRACK 1: C 영역을 위한 Cppcheck 설정 (MISRA C:2023 준수)
    # =================================================================
    find_program(CPPCHECK_EXECUTABLE cppcheck)
    if(CPPCHECK_EXECUTABLE)
        # deamon 폴더 내부의 misra_rules.txt 경로를 조준하여 json 생성
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/misra.json
            "{\n"
            "  \"script\": \"misra.py\",\n"
            "  \"args\": [\n"
            "    \"--rule-texts=${CMAKE_CURRENT_SOURCE_DIR}/deamon/misra_rules.txt\"\n"
            "  ]\n"
            "}\n"
        )

        set(CMAKE_C_CPPCHECK ${CPPCHECK_EXECUTABLE}
            --enable=warning,style,performance,portability
            "--template=VEDA_RAW|{severity}|{file}|{line}|{id}|{cwe}|{message}"
            --addon=${CMAKE_CURRENT_BINARY_DIR}/misra.json
            --suppressions-list=${CMAKE_CURRENT_SOURCE_DIR}/deamon/cppcheck_suppressions.txt
            --inline-suppr
            --quiet
        )
    endif()

    # =================================================================
    # TRACK 2: C++ 영역을 위한 Clang-Tidy 설정
    # =================================================================
    find_program(CLANG_TIDY_EXECUTABLE clang-tidy)
    if(CLANG_TIDY_EXECUTABLE)
        message(STATUS "🚀 Clang-Tidy 장착 완료!")
        set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXECUTABLE}
            "-checks=-*,bugprone-*,cert-*,cppcoreguidelines-*"
        )
    endif()
endif()

# =================================================================
# 3. src/ 폴더 내의 모든 C/C++ 소스코드 자동 수집 (무인 자동화)
# =================================================================
file(GLOB_RECURSE ALL_PROJECT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

if(NOT ALL_PROJECT_SOURCES)
    message(FATAL_ERROR "❌ src/ 폴더 내에 분석할 소스 코드가 하나도 없습니다!")
endif()

# 기존의 add_executable(veda_adts_target ${ALL_PROJECT_SOURCES}) 부분을 지우고 아래로 교체

# 각 소스 파일을 독립적인 실행 파일로 빌드
foreach(SOURCE_FILE ${ALL_PROJECT_SOURCES})
    # 파일 경로에서 파일명 추출 (예: src/test_c.c -> test_c)
    get_filename_component(TARGET_NAME ${SOURCE_FILE} NAME_WE)
    
    # 각 파일을 독립 실행 파일로 등록
    add_executable(${TARGET_NAME} ${SOURCE_FILE})
    
    message(STATUS "🛠️ 타겟 생성됨: ${TARGET_NAME}")
endforeach()
