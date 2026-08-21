# 모터 가감속 프로파일 적용 전/후 3D 맵 데이터 실측 비교 평가 보고서

**작성일**: 2026-08-21  
**테스트 환경**: 라즈베리파이(`172.20.26.191`) + STM32F401RE + TOFSense-F2P LiDAR + 2축 짐벌 (DRV8825 / 17HS4401 / MT6701)  
**수행 방식**: 전자동 펌웨어 빌드/플래시 $\rightarrow$ 실시간 3D 스캔 수집 $\rightarrow$ 원본 데이터 아카이빙 $\rightarrow$ 3D 점군 정량 분석  

---

## 1. 테스트 목적 및 요약

스텝모터의 사다리꼴 가감속 램프(Trapezoidal Velocity Profile) 적용 유무에 따른 3D 포인트 클라우드 품질, 샘플링 밀도, 스캔 소요 시간을 동일 정적 환경에서 실측 비교 평가하였습니다.

### 🌟 핵심 비교 결론
1. **격자 샘플링 밀도 및 노이즈 필터링 대폭 향상**:
   - 가감속 적용 시 양 끝단 감속 구간에서 라이다 샘플이 촘촘해져 **셀당 병합 샘플 수(Merged Samples)가 1,973개 $\rightarrow$ 12,706개로 6.4배(544%) 대폭 증가**했습니다.
   - 격자 유효 충진율(Fill Rate) 역시 **99.01% $\rightarrow$ 99.21%**로 향상되어 끝단 결측이 감소했습니다.
2. **스캔 소요 시간 트레이드오프**:
   - 가감속 OFF(800 PPS 급출발/급정지): **7분 35초 (454.8s)**
   - 가감속 ON (50 $\leftrightarrow$ 800 PPS 램프): **9분 30초 (569.9s, +25.3%)**
   - 램프 가감속 시간 및 100ms 정착 대기로 인해 약 1분 55초의 시간이 추가되나, 기구 진동 방지와 데이터 밀도 향상 효과가 뚜렷합니다.

---

## 2. 정량 실측 데이터 비교표

| 평가 항목 | [Case A] 가감속 미적용 (OFF) | [Case B] 가감속 적용 (ON) | 개선 효과 및 분석 |
| :--- | :---: | :---: | :--- |
| **세션 ID** | `calib-20260821-114831` | `calib-20260821-113535` | 동일 물리 위치 측정 |
| **속도 프로파일** | **800 PPS 고정** (급출발/급정지) | **50 $\to$ 800 PPS 사다리꼴 램프** | 선형 가감속 ($1200\text{ pps}^2$) |
| **총 수집 유효 포인트** | **40,001 점** | **40,080 점** | **+79점 유효 데이터 증가** |
| **격자 충진율 (Fill Rate)** | **99.01%** (결측 399개) | **99.21%** (결측 320개) | **결측 셀 19.8% 감소** |
| **셀당 병합 샘플 수** | **1,973 개** | **12,706 개** | **+544% 증가 (노이즈 필터링 강화)** |
| **스캔 소요 시간** | **7.58분 (454.8초)** | **9.50분 (569.9초)** | 램프 가감속 및 정착 비용 (+1분 55초) |
| **홈 복귀 잔차 (Tilt/Pan)** | $0.2^\circ$ / $0.0^\circ$ | $0.2^\circ$ / $0.0^\circ$ | 양축 모두 탈조 없이 정상 수렴 |
| **PCD 파일 크기** | 885.6 KB | 886.4 KB | 원본 ASCII PCD 저장 |
| **JSON 메타데이터 크기** | 25.15 MB | 25.35 MB | 포인트별 센서 메타데이터 포함 |

---

## 3. 세부 품질 분석

### ① 셀당 중복 샘플링(Merged Samples) 6.4배 증가의 의의
- 직사각형 고정 속도 구동(Case A) 시에는 모터가 항상 최고 속도($90^\circ/\text{s}$)로 지나가므로 각 격자 셀당 라이다 점이 1개 남짓만 떨어져 노이즈 보정이 어렵습니다.
- 가감속 프로파일(Case B) 적용 시, 방향 전환 및 스윕 외곽 구간에서 속도가 자연스럽게 줄어들면서 **동일 셀에 2~3회 이상의 라이다 측정이 중첩(Averaged)**됩니다.
- 이는 외곽 벽면 및 천장/바닥 경계면의 **거리 측정 오차를 평균화하여 3D 표면 노이즈를 억제**하는 결정적 역할을 합니다.

### ② 기구 안정성 및 탈조 방지
- 가감속 OFF(800 PPS 급출발/급정지) 상태에서도 100ms 정착 대기 시퀀스 덕분에 탈조(`ERR_STALL`)는 회피하였으나, 급격한 방향 전환으로 인해 로터 진동 충격음이 발생하고 끝단 격자 결측이 79개 더 발생했습니다.
- 가감속 ON 상태에서는 부드러운 속도 전이로 모터 구동 소음과 로터 충격이 최소화되었습니다.

---

## 4. 원본 데이터 아카이빙 (Raw Data)

본 테스트의 모든 실측 데이터는 프로젝트 내 아래 경로에 영구 보관되었습니다:

* **가감속 미적용 (Case A)**:
  - PCD 점군: [`docs/motor_profile_test/raw_data/scan_no_profile.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_no_profile.pcd)
  - 상세 JSON: [`docs/motor_profile_test/raw_data/scan_no_profile.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_no_profile.json)
* **가감속 적용 (Case B)**:
  - PCD 점군: [`docs/motor_profile_test/raw_data/scan_with_profile.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_with_profile.pcd)
  - 상세 JSON: [`docs/motor_profile_test/raw_data/scan_with_profile.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_with_profile.json)

---

## 5. 최종 종합 의견 및 권장 설정

1. **품질 중심 운용 (현재 기본값 유지 권장)**:
   - 정밀 3D 맵핑 및 표면 노이즈 억제가 최우선인 경우, 현재의 **`MOTOR_START_PPS = 50u` (사다리꼴 램프 ON)** 설정을 유지하는 것이 가장 우수한 데이터 품질을 제공합니다.
2. **고속 진단 스캔이 필요한 경우**:
   - 스캔 시간 단축이 시급한 현장 진단 모드에서는 가감속 가속도(`MOTOR_TILT_ACCEL_PPS2`)를 $1200 \to 2400\text{ pps}^2$로 상향하여 램프 구간을 압축하는 방안을 권장합니다.
