# A.D.T.S — STM32 Firmware (`adts`)

**Anti-Drone Tracking & Targeting System** 의 STM32 펌웨어.
RPi(엣지 서버)와 UART로 통신하며, 2축 스텝모터(방위/고각)·리밋 스위치·라이다(TOFSense-F2 P)를 실시간 제어한다.

- **MCU**: STM32F401RE (NUCLEO)
- **빌드**: CMake + arm-none-eabi-gcc (CubeMX의 CMake 생성)
- **RPi 링크**: USART1 (PA9 TX / PA10 RX), 115200 8N1
- **디버그**: USART2 (ST-Link VCP, `printf` 출력)

---

## 📂 디렉토리 구조

```
.
├── adts.ioc                 # CubeMX 설정 (HW single source — 반드시 커밋)
├── CMakeLists.txt           # 프로젝트 CMake (유저 소스/인클루드 등록)
├── CMakePresets.json        # Debug/Release 프리셋
├── cmake/                   # arm-none-eabi 툴체인 파일
├── startup_*.s  *.ld        # 스타트업 / 링커 스크립트
│
├── Core/                    # CubeMX 생성 (main.c 얇게 유지)
│   ├── Inc/  Src/
├── Drivers/                 # HAL / CMSIS (벤더)
│
├── App/                     # ★ 우리 앱 로직 (CubeMX 재생성에도 안전)
│   ├── uart_rpi/            #   RPi UART 포트제어·프로토콜 디스패처  (이현우)
│   ├── motor/               #   스텝모터·리밋 홈 캘리브레이션        (강유근)  ⏳
│   └── lidar/               #   TOFSense-F2 P NLink 파서             (송영빈)  ⏳
│
├── shared/
│   └── protocol.h           # ★ RPi↔STM32 통신 계약 (rpi repo에서 동기화)
│
├── tools/                   # 정적분석 설정
│   ├── cppcheck_suppressions.txt
│   ├── misra_rules.txt
│   └── run_static_analysis.sh
│
├── .github/workflows/       # CI (정적분석 게이트)
└── host_ci/                 # (임시) 호스트측 CI 하네스 — 추후 rpi repo 이관
```

### 설계 원칙: 3층 분리
- **`Core/` · `Drivers/`** = CubeMX/벤더 생성. `.ioc` 재생성 시 덮어써짐 → 거의 안 건드린다.
- **`App/`** = 우리 앱 로직. CubeMX가 안 건드리므로 여기서 모듈 개발.
- **`shared/`** = 통신 계약(`protocol.h`). RPi 드라이버와 **동일 파일**을 공유한다.

`main.c` 는 얇게 유지한다 — 초기화/메인루프/HAL 콜백에서 `App/` 모듈 함수만 호출한다.

---

## 🔨 빌드

### 요구 도구
- `arm-none-eabi-gcc` (권장 15.2 / Arm GNU Toolchain)
- `cmake` (≥3.22), `ninja`
- (편의) STM32CubeCLT 또는 STM32CubeIDE 1.15+

### 명령
```bash
# 구성 + 빌드 (Debug 프리셋)
cmake --preset Debug
cmake --build build/Debug          # → build/Debug/adts.elf

# 클린
rm -rf build
```

### 플래시 / 디버그
- **CubeIDE**: `adts.ioc` (또는 CMake 프로젝트) import 후 Run/Debug.
- **CLI**: `STM32_Programmer_CLI -c port=SWD -w build/Debug/adts.elf -rst`
- **VCP 로그**: USART2가 ST-Link VCP. `printf` 출력을 시리얼 터미널(115200)로 확인.
  - macOS: `/dev/cu.usbmodem*` 사용 (`/dev/tty.*` 아님).

---

## 🛡️ 정적분석 (로컬에서 push 전 검사)

```bash
bash tools/run_static_analysis.sh      # repo 루트에서 실행
```
- `cmake --preset Debug` 로 `compile_commands.json` 생성 → `cppcheck` 실행.
- **우리 코드(`App/`, `Core/Src/main.c`)만** 검사. 벤더/자동생성은 `tools/cppcheck_suppressions.txt` 로 제외.
- CMSIS `#error Unknown compiler` 는 `-D__GNUC__` 정의로 회피.
- **지적사항이 있으면 exit 1** → CI 게이트로 머지 차단.

CI(`.github/workflows/static_analysis.yml`)가 push/PR 시 자동 실행한다.

---

## 🔗 protocol.h 동기화 규칙 (중요)

`shared/protocol.h` 는 **RPi↔STM32 통신 계약**이며 **단일 원본은 `rpi` repo(`rpi/shared/protocol.h`)** 다.
이 repo의 사본은 **다운스트림**이다.

- 프로토콜을 바꿔야 하면 → **`rpi` repo에서 먼저 수정**하고, 이 사본을 맞춘다.
- CI의 **drift-check** 가 두 파일이 다르면 PR을 **차단**한다 → 조용한 버전 불일치(CRC/디코드 오류) 방지.
- 프레임: `[SOF(0xAA)][CMD][LEN][PAYLOAD][CRC16]`, CRC-16/CCITT-FALSE, `PROTO_VERSION=3`.

---

## 👥 모듈 담당 (CODEOWNERS)

| 경로 | 담당 |
|---|---|
| `App/uart_rpi/`, `shared/` | 이현우 |
| `App/motor/` | 강유근 |
| `App/lidar/`, 메인루프·IWDG | 송영빈 |
| `tools/`, `.github/` | 강유근 (QA) |

---

## ⚠️ 주의사항

- **`.ioc` 는 반드시 커밋** (HW 설정 single source). `Core/`·`Drivers/` 생성물도 커밋해 재생성 없이 동일 빌드 보장.
- **`build/`, `compile_commands.json`, `.mxproject`, `.idea/` 는 커밋 금지** (`.gitignore` 처리됨).
- `.ioc` 는 **단일 파일 = 편집 병목**. 페리페럴 추가는 한 사람이 순차적으로(PR) — 동시 편집 시 merge 충돌.
- 툴체인 버전(arm-none-eabi-gcc)은 팀이 통일한다.
