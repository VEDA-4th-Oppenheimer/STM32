# STM32 Firmware (`adts`)

1D LiDAR Pan-Tilt 스캐너의 실시간 제어 펌웨어. RPi 와 UART 로 통신하며 2축
스텝모터·MT6701 엔코더·라이다(TOFSense-F2 P)를 다룬다.

RPi 가 스캔 범위를 내리면 펌웨어가 격자로 스윕하며, 라이다 프레임이 완성되는
순간 각도를 래치해 `(pan, tilt, d)` 를 스트림으로 올린다.

| | |
|---|---|
| MCU | STM32F401RE (NUCLEO) |
| 빌드 | CMake + arm-none-eabi-gcc (CubeMX 의 CMake 생성) |
| RPi 링크 | USART1 (PA9 TX / PA10 RX), 115200 8N1 |
| 라이다 | USART6, NLink 액티브 100Hz |
| 엔코더 | I2C3 (Pan) · I2C1 (Tilt), MT6701 14비트, 100kHz |
| 모터 | TIM1 (Pan) · TIM2 (Tilt), DRV8825, 1/16 마이크로스텝 |
| 디버그 | USART2 (ST-Link VCP, `printf`) |

이름의 `A.D.T.S`(Anti-Drone…)는 2026-07-22 주제 전환 이전 것이다.

---

##  디렉토리 구조

```
.
├── adts.ioc                 # CubeMX 설정 (HW single source — 반드시 커밋)
├── CMakeLists.txt           # 유저 소스/인클루드 등록
├── CMakePresets.json        # Debug/Release 프리셋
├── cmake/                   # arm-none-eabi 툴체인 파일
│
├── Core/                    # CubeMX 생성 (main.c 얇게 유지)
├── Drivers/                 # HAL / CMSIS (벤더)
│
├── App/                     # 우리 앱 로직 (CubeMX 재생성에도 안전)
│   ├── uart_rpi/            #   RPi UART 포트제어·프로토콜 디스패처  (이현우)
│   ├── scan/                #   스캔 시퀀서 (홈·스윕·파킹 상태머신)  (이현우)
│   ├── motor/               #   스텝모터 구동·가감속 램프            (강유근)
│   ├── hallEffectSensor/    #   MT6701 엔코더 판독·I2C 버스 복구     (강유근)
│   ├── lidar/               #   TOFSense-F2 P NLink 파서             (송영빈)
│   └── bench_common.h       #   브링업 벤치 공용 (워치독·UART 서비스)
│
├── shared/
│   └── protocol.h           # RPi↔STM32 통신 계약 (rpi repo 에서 동기화)
│
├── tools/
│   ├── flash.sh             #   빌드 → RPi 전송 → SWD 플래시 (검증 포함)
│   ├── run_static_analysis.sh
│   ├── cppcheck_suppressions.txt
│   └── misra_rules.txt  misra.json
│
└── .github/workflows/       # CI (정적분석 + protocol drift-check)
```

### 설계 원칙: 3층 분리

- **`Core/` · `Drivers/`** = CubeMX/벤더 생성. `.ioc` 재생성 시 덮어써진다.
- **`App/`** = 우리 앱 로직. CubeMX 가 안 건드리므로 여기서 개발한다.
- **`shared/`** = 통신 계약. RPi 마스터에서 내려온 사본이다.

`main.c` 는 얇게 유지한다 — 초기화·메인루프·HAL 콜백에서 `App/` 함수만 부른다.

### 실시간성 분리

스캔 시퀀서는 **메인루프**에 있고 스텝 펄스와 각도 래치는 **ISR** 에 있다.
그래서 메인루프가 UART 송신으로 잠깐 막혀도 모터 타이밍과 `(각도, 거리)` 짝은
흔들리지 않는다.

시퀀서가 ISR 안에 있던 시절에는 거기서 `HAL_Delay`(SysTick 대기 → 데드락)와
블로킹 I2C(라이다 프레임 유실)를 했다. 개별 버그가 아니라 계층 문제였다.

---

##  빌드

### 요구 도구

- `arm-none-eabi-gcc` (권장 15.2 / Arm GNU Toolchain)
- `cmake` (≥3.22), `ninja`
- (편의) STM32CubeCLT 또는 STM32CubeIDE 1.15+

### 명령

```bash
cmake --preset Debug
cmake --build build/Debug          # → build/Debug/adts.elf
rm -rf build                       # 클린
```

빌드 로그에 `#warning` 이 하나도 없어야 정상이다. 브링업 플래그(아래)를 켜면
일부러 뜬다.

---

##  플래시

### tools/flash.sh — RPi 를 플래시 호스트로

Mac 에서 ST-Link 가 안 잡히는 문제 때문에 **Pi 에 꽂힌 ST-Link 로 굽는다.**
빌드 → `scp` → OpenOCD SWD 플래시 → 검증까지 한 번에 한다.

```bash
bash tools/flash.sh                  # 빌드 → 전송 → 플래시 → 검증
bash tools/flash.sh -n               # 빌드 없이 지금 elf 그대로
bash tools/flash.sh -H pi@10.0.0.5   # 호스트 직접 지정
bash tools/flash.sh -l -e ~/adts.elf # Pi 에서 직접 (로컬 모드)
```

| 옵션 | |
|---|---|
| `-H <host>` | RPi ssh 대상 (기본 `adts-pi`) |
| `-e <경로>` | 플래시할 `.elf` (기본 `build/Debug/adts.elf`) |
| `-l` | 로컬 모드 — Pi 에서 실행할 때 |
| `-n` | 빌드 건너뛰기 |

성공 신호는 **`** Verified OK **`** 다.

**드래그앤드롭(MSD)이 아니라 SWD 를 쓰는 이유는 검증이다.** NUCLEO 의 USB
드라이브에 `.bin` 을 `cp` 하면 거의 항상 성공을 반환하는데, 그건 데이터를
장치에 넘겼다는 뜻이지 플래시에 앉았다는 뜻이 아니다. 읽어서 대조하지 않으므로
부분 기록이 나도 모른다. `.elf` 를 그대로 써서 `objcopy` 단계도 없앴다 — 옛
`.bin` 을 올려놓고 "코드를 고쳤는데 왜 그대로지" 하는 사고를 막는다.

사전 준비:

```bash
# Pi
sudo apt install openocd
# Mac (한 번만, 안 하면 매번 비밀번호를 묻는다)
ssh-copy-id adts-pi
```

### 그 밖의 방법

- **CubeIDE**: `adts.ioc` 또는 CMake 프로젝트 import 후 Run/Debug.
- **CLI 직결**: `STM32_Programmer_CLI -c port=SWD -w build/Debug/adts.elf -v -rst`
- **VCP 로그**: USART2 가 ST-Link VCP. 시리얼 터미널 115200.
  macOS 는 `/dev/cu.usbmodem*` (`/dev/tty.*` 아님), Pi 는 `/dev/ttyACM0`.

---

##  브링업 플래그

하드웨어 없이, 또는 계통별로 진단하려고 두는 컴파일타임 스위치다. 헤더의 값
하나(`0` ↔ `1`)만 바꾸면 되고, 켜면 빌드마다 `#warning` 이 떠서 지나칠 수 없다.

| 플래그 | 위치 | 하는 일 |
|---|---|---|
| `SCAN_NO_ENCODER` | `App/scan/scan.h` | 판독 없이 "지금 자리 = 기구각 0" 선언. 산출물의 엔코더 raw 가 `0xFFFF` 로 나가 구분된다 |
| `ENCODER_BENCH_TEST` | `App/hallEffectSensor/encoder_bench.h` | 풀업 실측·라인 4분류·주소 스캔·판독방식 탐색 |
| `MOTOR_BENCH_TEST` | `App/motor/motor_bench.h` | 축 왕복 + STEP 펄스 실제 출력 증명 |
| `LIDAR_BENCH_TEST` | `App/lidar/lidar_bench.h` | 바이트·프레임·체크섬오류·drop 카운터 + 레이트 |

**전부 `0` 이어야 정상 모드다.** 켜둔 채 잊으면 데모에서 그대로 나간다.

벤치가 메인루프를 오래 잡으면 PONG 이 300ms 를 넘겨 RPi 가 `link_dead` 로
판정한다. `BENCH_SERVICE()` 가 워치독과 `uart_rpi_process()` 를 같이 도는 이유다.

---

##  정적분석 (push 전 로컬 검사)

```bash
bash tools/run_static_analysis.sh      # repo 루트에서
```

- `cmake --preset Debug` 로 `compile_commands.json` 생성 → `cppcheck` + MISRA.
- **우리 코드(`App/`)만** 검사한다. 벤더·자동생성은 `tools/cppcheck_suppressions.txt`
  로 제외한다. `main.c` 도 제외 — 생성 코드와 USER CODE 가 섞여 부분검사가 안 된다.
  그래서 `main.c` 의 USER CODE 는 `App/` 호출만 두는 규율이 필요하다.
- CMSIS `#error Unknown compiler` 는 `-D__GNUC__` 로 회피한다.
- 지적사항이 있으면 exit 1 → CI 게이트로 머지 차단.

**MISRA 게이트 정책은 A** — Required/Mandatory 만 차단하고 Advisory 는 근거를 달아
deviation 처리한다. 폴더를 통째로 억제하지 않는다(그렇게 해서 `App/motor` 가 한동안
검사에서 빠져 있었다).

로컬 통과가 CI 통과를 보장하지 않는다. CI 의 cppcheck 는 2.13 이라 로컬(2.21)에
없는 오탐이 나온다 — MISRA 11.8 이 대표적이고, 근거를 달아 억제해 두었다.

CI 는 `main` 브랜치에만 걸린다. required check 는 `firmware-analysis` +
`protocol-sync-check` 둘이다.

---

##  protocol.h 동기화 규칙

`shared/protocol.h` 는 RPi↔STM32 통신 계약이며 **단일 원본은 `RPi` repo 의
`shared/protocol.h`** 다. 이 repo 의 것은 **사본**이다.

- 프로토콜을 바꾸려면 **RPi repo 에서 먼저** 고치고 이 사본을 맞춘다.
- **push 순서**: RPi `main` 을 먼저 반영한 뒤 STM32 를 push 한다. drift-check 가
  RPi `main` 의 raw 를 보므로 역순이면 이 repo 의 PR 이 막힌다.
- CI 의 drift-check 가 두 파일이 다르면 PR 을 차단한다.

현재 **`PROTO_VERSION = 6`**. 프레임은 `[SOF(0xAA)][CMD][LEN][PAYLOAD][CRC16]`,
CRC-16/CCITT-FALSE.

---

## 주의

- **`.ioc` 는 반드시 커밋**한다 (HW 설정 single source). `Core/`·`Drivers/` 생성물도
  커밋해 재생성 없이 동일 빌드를 보장한다.
- `.ioc` 는 **단일 파일 = 편집 병목**이다. 페리페럴 추가는 한 사람이 순차적으로.
- **CubeMX 생성 파일은 CRLF 다.** 편집 도구가 줄바꿈을 바꾸면 diff 가 수백 줄로
  터지고 재생성·머지에서 충돌한다.
- `build/`·`compile_commands.json`·`.mxproject` 커밋 금지.
- **모터를 만지기 전에 VMOT 를 끈다.** 통전 중 코일 착탈은 역기전력으로 드라이버를
  태운다 — DRV8825 가 죽는 가장 흔한 원인이다.
- 툴체인 버전(arm-none-eabi-gcc)은 팀이 통일한다.
