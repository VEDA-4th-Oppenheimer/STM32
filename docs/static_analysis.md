# STM32 펌웨어 정적분석 가이드

이 레포(STM32 펌웨어) 기준 문서다. RPi 드라이버·데몬(clang-tidy, 하드닝 플래그)은
별도 레포 소관이라 여기서 다루지 않는다.

## 1. 실행

```bash
bash tools/run_static_analysis.sh
```

CubeMX CMake 프리셋(Debug)으로 `compile_commands.json` 을 재생성한 뒤 cppcheck 를
돌린다. CMSIS 의 `#error Unknown compiler` 를 피하려고 `-D__GNUC__=15` 를 주입하고,
지적이 하나라도 있으면 `--error-exitcode=1` 로 빌드를 차단한다.

## 2. 게이트 정책 A

Required/Mandatory 만 게이트하고 Advisory 는 리포트만 한다. cppcheck misra 애드온에
등급 필터가 없어서, 실제로 뜨는 Advisory 룰을 근거를 달아 개별 등록하는 방식으로
운영한다. **폴더 통째 억제로 되돌리지 말 것** — 룰 단위 deviation 을 유지한다.

## 3. 억제 목록 (`tools/cppcheck_suppressions.txt`)

| 구분 | 항목 | 사유 |
|---|---|---|
| 벤더/생성물 | `*:*Core/*`, `*:*Drivers*` | CubeMX·HAL·CMSIS. 우리가 MISRA 를 고칠 대상이 아니다 |
| 시스템 노이즈 | `missingIncludeSystem`, `preprocessorErrorDirective`, `misra-config` | 파서 설정 관련 |
| `shared/protocol.h` | 2.3 / 2.4 / 2.5 / 20.1 | RPi·커널·STM 이 공유하는 계약이라 특정 컴포넌트가 안 쓰는 심볼은 정상 |
| `App/*` | 11.4 | ST HAL 페리페럴 베이스가 `(GPIO_TypeDef*)BASE` 형태라 구조적으로 발생 |
| `App/*` | 8.7 | 모듈 공개 API 를 "한 파일에서만 쓴다" 고 보는 오탐 |
| `encoder_bench.c/.h` | 21.6 / 2.5 | 벤치 브링업 도구. `ENCODER_BENCH_TEST=0`(기본) 이면 컴파일아웃 |
| `App/motor/motor.c` | 11.8 | cppcheck 2.13.0 오탐 (아래 5절) |

### Core/ 를 통째로 제외하는 이유

`main.c` 는 CubeMX 생성코드와 USER CODE 가 한 파일에 섞여 부분검사가 불가능하다.
대신 규율로 대응한다 — **main.c 의 USER CODE 에는 App/ 함수 호출(얇은 글루)만 두고,
실제 로직은 App/ 에 작성해 정적검사를 받는다**(§11 컨벤션).

파일 이름을 하나씩 나열하는 방식으로 바꾸면 `Core/Src/main.c` 와 `Core/Inc/*.h` 가
빠지기 쉽다. 실측으로 지적이 0건에서 195건이 된다.

### 억제파일 작성 규칙

구분선에 `#` 단독 줄을 쓰지 말 것. cppcheck 2.13 파서는 길이 2 이상일 때만 주석으로
처리하므로 `#` 한 글자는 규칙으로 파싱돼 `Failed to add suppression. No id.` 로 CI 가
실패한다. `# ---` 처럼 내용을 붙인다. (9d2d120 에서 고쳤다가 두 번 재발한 이력이 있다)

## 4. CI

`.github/workflows/static_analysis.yml` — `main` 대상 push/PR 에서 실행.
필수 상태 체크는 `firmware-analysis`, `protocol-sync-check` 두 개이고, `main` 은
브랜치 보호로 **PR 을 통해서만** 변경할 수 있다(직접 push 불가).

## 5. 알려진 문제 — cppcheck 버전 불일치

워크플로가 `apt-get install -y cppcheck` 로 버전을 고정하지 않아 러너(ubuntu noble)
에서는 **2.13.0** 이 설치된다. 로컬 개발기가 더 최신이면 같은 코드에서 결과가 갈린다.
"로컬은 통과하는데 CI 만 실패" 하면 먼저 이걸 의심할 것. 설치 스텝이
`cppcheck --version` 을 찍으므로 로그에서 확인할 수 있다.

2.13.0 에서만 나타나는 것으로 확인된 두 가지:

- 억제파일의 `#` 단독 줄을 규칙으로 파싱 (3절 참조)
- `misra-c2012-11.8` 오탐 — `App/motor/motor.c` 의 `HAL_GPIO_WritePin` 호출 8곳.
  `s_cfg` 가 `static const` 라 멤버 lvalue 는 `GPIO_TypeDef *const` 지만, 값을 읽으면
  top-level const 는 사라져 `GPIO_TypeDef *` 가 된다. HAL 파라미터도 같은 타입이라
  가리키는 타입은 양쪽 다 무자격 — 제거되는 자격이 없다. 11.8 은 pointed-to 자격만
  다루는 룰이므로 위반이 아니다. cppcheck 2.21 에서는 0건이다.

**미해결 과제**: CI cppcheck 버전 고정. noble apt 에는 2.13.0 뿐이라 상향하려면
소스 빌드나 컨테이너 도입이 필요하고 방식이 정해지지 않았다. 2.14 이상으로 올리면
억제파일 7절의 11.8 deviation 은 삭제 대상이다.
