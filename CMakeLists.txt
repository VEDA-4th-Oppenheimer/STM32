cmake_minimum_required(VERSION 3.10)
project(VEDA_ADTS_PROJECT C CXX)

# 1. 정적 분석 옵션 설정
option(ENABLE_STATIC_ANALYSIS "Enable static analysis tools" OFF)

if(ENABLE_STATIC_ANALYSIS)
    find_program(CPPCHECK_PATH cppcheck)
    if(CPPCHECK_PATH)
        # deamon/ 폴더에 위치한 suppressions 파일 경로 지정
        set(CPPCHECK_SUPPRESSIONS "${CMAKE_SOURCE_DIR}/deamon/cppcheck_suppressions.txt")
        
        # Cppcheck 실행 인수 설정
        set(CPPCHECK_ARGS 
            "--enable=warning,style,performance,portability"
            "--suppressions-list=${CPPCHECK_SUPPRESSIONS}"
            "--inconclusive"
            "--force"
            "--inline-suppr"
        )
        
        # C 및 C++ 타겟에 Cppcheck 적용
        set(CMAKE_C_CPPCHECK "${CPPCHECK_PATH}" ${CPPCHECK_ARGS})
        set(CMAKE_CXX_CPPCHECK "${CPPCHECK_PATH}" ${CPPCHECK_ARGS})
    endif()
endif()

# 2. 실행 파일 타겟 생성 (tree 구조 반영)

# test1 폴더의 소스
add_executable(add_push_main src/test1/add_push_main.c)

# test2 폴더의 소스
add_executable(test_success_c src/test2/test_success_c.c)
add_executable(test_success_c src/test3/test_c.c)

