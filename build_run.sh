#!/bin/bash
# ANSI 컬러 코드 정의
RED='\033[1;31m'
YELLOW='\033[1;33m'
GREEN='\033[1;32m'
CYAN='\033[1;36m'
GRAY='\033[1;30m' # 두꺼운 회색 (참고 노트용)
RESET='\033[0m'

echo "============================================="
echo "🚀 VEDA 프로젝트 빌드 및 정적 분석(2-Track) 시작"
echo "============================================="

# [옵션 확인] clean 인자가 들어오면 build 폴더 삭제
if [ "$1" == "clean" ]; then
    echo "🧹 [옵션 확인] 기존 빌드 디렉토리(build)를 완전히 초기화합니다..."
    rm -rf build
fi

echo "📂 [1/2] CMake 환경 구성 중..."

# 빌드 디렉토리가 없으면 생성
mkdir -p build

# CMake 환경 구성 진행
cmake -B build -S .

# CMake 구성 실패 시 스크립트 종료
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ CMake 환경 구성 중 에러가 발생했습니다.${RESET}"
    exit 1
fi

echo "🛠️ [2/2] 소스 코드 컴파일 및 2-Track 분석 구동 중..."

# 컴파일 및 정적 분석을 수행하며 출력을 실시간으로 필터링
cmake --build build 2>&1 | while read -r line; do
    
    # -----------------------------------------------------------------
    # TRACK 1: Cppcheck (C 영역 - MISRA / CWE 사령부)
    # -----------------------------------------------------------------
    if [[ "$line" == "VEDA_RAW"* ]]; then
        IFS='|' read -r prefix severity file line_num id cwe message <<< "$line"
        
        id=$(echo "$id" | xargs)
        cwe=$(echo "$cwe" | xargs)
        
        if [[ "$id" == *"misra"* ]]; then
            echo -e "${YELLOW}[MISRA C:2023 위반][${severity}]${RESET} ${file}:${line_num}: ${message} (${id})"
        elif [[ -n "$cwe" && "$cwe" != "0" ]]; then
            echo -e "${RED}[CWE 보안위반][${severity}]${RESET} ${file}:${line_num}: ${message} (CWE-${cwe})"
        else
            echo -e "${GREEN}[일반 정적분석][${severity}]${RESET} ${file}:${line_num}: ${message} (${id})"
        fi

    # -----------------------------------------------------------------
    # TRACK 2: 컴파일러 자체 경고(GCC/G++) 및 Clang-Tidy 오버랩 방지 분기
    # -----------------------------------------------------------------
    # 패턴 정규식: 파일:라인:열: warning/error: 메시지 [규칙이름]
    elif [[ "$line" =~ ^(.+):([0-9]+):[0-9]+:[[:space:]](warning|error):[[:space:]](.*)\[([^\]]+)\]$ ]]; then
        file="${BASH_REMATCH[1]}"
        line_num="${BASH_REMATCH[2]}"
        severity="${BASH_REMATCH[3]}"
        msg="${BASH_REMATCH[4]}"
        check="${BASH_REMATCH[5]}"
        
        clean_file=$(basename "$file")

        # 확장자가 .c 로 끝나는 경우 순수 C 컴파일러 경고로 처리
        if [[ "$file" == *.c ]]; then
            echo -e "${RED}[GCC 컴파일러 경고][${severity}]${RESET} src/${clean_file}:${line_num}: ${msg} (${check})"
        else
            # 그 외 C++ 파일(.cpp, .cc)은 Modern C++ 가이드라인으로 처리
            if [[ "$severity" == "error" ]]; then
                echo -e "${RED}[C++ 가이드 치명오류][error]${RESET} src/${clean_file}:${line_num}: ${msg} (${check})"
            else
            	echo -e "${CYAN}[C++ 분석][${severity}]${RESET} src/${clean_file}:${line_num}: ${msg} ${YELLOW}(Rule: ${check})${RESET}"
	    fi
        fi

    # 컴파일러 및 린터의 상세 설명(note:) 라인 파싱
    elif [[ "$line" =~ ^(.+):([0-9]+):[0-9]+:[[:space:]]note:[[:space:]](.*)$ ]]; then
        file="${BASH_REMATCH[1]}"
        line_num="${BASH_REMATCH[2]}"
        msg="${BASH_REMATCH[3]}"
        
        if [[ "$file" == *.c ]]; then
            echo -e "   ${GRAY}└─ [컴파일러 참고노트] (line:${line_num}) ${msg}${RESET}"
        else
            echo -e "   ${GRAY}└─ [C++ 분석 참고노트] (line:${line_num}) ${msg}${RESET}"
        fi

    # -----------------------------------------------------------------
    # 3. 기타 일반 빌드 메시지 (진행률 %, 링킹 메시지 등)
    # -----------------------------------------------------------------
    else
        echo "$line"
    fi
done

echo "============================================="
echo "🎉 빌드 및 정적 분석이 완료되었습니다!"
echo "============================================="
