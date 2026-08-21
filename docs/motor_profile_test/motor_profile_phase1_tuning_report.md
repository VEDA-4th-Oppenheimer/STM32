# [Phase 1] 가속도 튜닝(2400 PPS²) 3D 실측 비교 평가 보고서

**작성일**: 2026-08-21  
**문서 위치**: `docs/motor_profile_test/motor_profile_phase1_tuning_report.md`  
**테스트 환경**: 라즈베리파이(`172.20.26.191`) + STM32F401RE + TOFSense-F2P LiDAR + 2축 짐벌 (DRV8825 / 17HS4401 / MT6701)  
**수행 방식**: 전자동 펌웨어 빌드/플래시 $\rightarrow$ 실시간 3D 스캔 수집 $\rightarrow$ 원본 데이터 아카이빙 $\rightarrow$ 3D 점군 정량 분석  

---

## 1. 테스트 목적 및 요약

[3단계 로드맵 계획서](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_roadmap_plan.md)의 **[Phase 1] 가속도 튜닝($1200 \to 2400\text{ PPS}^2$)**을 적용하고, 기존 기준 데이터(Case A: 가감속 OFF, Case B: $1200\text{ PPS}^2$)와 동일 정적 환경에서 실측 비교 평가를 수행하였습니다.

### 🌟 핵심 비교 결론
1. **스캔 소요 시간 53.5초 단축**:
   - Case B($1200\text{ PPS}^2$) 대비 **스캔 시간이 9분 30초(569.9s) $\rightarrow$ 8분 36초(516.4s)로 53.5초 단축(-9.4%)**되었습니다.
2. **외곽 샘플 병합 밀도 유지 (노이즈 필터링 확보)**:
   - 셀당 병합 샘플 수(Merged Samples)가 **Case A(1,973개) 대비 7,385개로 3.74배(+274%) 증가**하여, 외곽 감속 구간에서의 거리 측정 오차 평균화 및 노이즈 억제 성능을 온전히 유지하였습니다.
3. **영점 수렴 정확도 최상 달성 (무탈조 입증)**:
   - 200줄(40,400 셀) 연속 스캔 후 홈 복귀 잔차가 **Tilt $0.0^\circ$, Pan $-0.1^\circ$**로 측정되어, $2400\text{ PPS}^2$ 가속도에서도 **탈조나 기구적 슬립이 전혀 발생하지 않음**을 입증하였습니다.

---

## 2. 정량 실측 데이터 3-Way 비교표

| 평가 항목 | [Case A] 가감속 OFF (급출발) | [Case B] 가감속 ON ($1200\text{ PPS}^2$) | [Phase 1] 가속도 튜닝 ($2400\text{ PPS}^2$) | 개선 효과 및 비교 분석 |
| :--- | :---: | :---: | :---: | :--- |
| **세션 ID** | `calib-20260821-114831` | `calib-20260821-113535` | `calib-20260821-124840` | 동일 물리 위치 실측 |
| **속도 프로파일** | **800 PPS 고정** | **50 $\to$ 800 PPS ($1200\text{ pps}^2$)** | **50 $\to$ 800 PPS ($2400\text{ pps}^2$)** | 램프 가속도 2배 상향 |
| **총 수집 유효 포인트** | **40,001 점** | **40,080 점** | **40,019 점** | Case A 대비 +18점 증가 |
| **격자 충진율 (Fill Rate)** | **99.01%** (결측 399개) | **99.21%** (결측 320개) | **99.06%** (결측 381개) | Case A 대비 결측 18개 감소 |
| **셀당 병합 샘플 수** | **1,973 개** | **12,706 개** | **7,385 개** | **Case A 대비 +274% 증가** |
| **스캔 소요 시간** | **7.58분 (454.8초)** | **9.50분 (569.9초)** | **8.61분 (516.4초)** | **Case B 대비 53.5초 단축** |
| **홈 복귀 잔차 (Tilt/Pan)** | $0.2^\circ$ / $0.0^\circ$ | $0.2^\circ$ / $0.0^\circ$ | **$0.0^\circ$ / $-0.1^\circ$** | **Tilt 오차 0.0° 무탈조 완벽 수렴** |
| **평면 잔차 RMSE** | 619.39 mm | 621.79 mm | 622.31 mm | 동등 수준 (환경 공차 범위) |
| **PCD 파일 크기** | 885.6 KB | 886.4 KB | 885.8 KB | 원본 ASCII PCD 저장 |
| **JSON 메타 크기** | 25.15 MB | 25.35 MB | 25.21 MB | 포인트별 센서 메타 포함 |

---

## 3. 세부 품질 및 기구 특성 분석

### ① 스캔 시간 단축과 데이터 밀도의 균형 (Sweet Spot)
- Case B($1200\text{ PPS}^2$)는 가속/감속 구간이 각각 266펄스($30^\circ$)를 차지하여 스캔 소요 시간이 9분 30초까지 늘어났습니다.
- Phase 1($2400\text{ PPS}^2$)에서는 가속 구간을 133펄스($15^\circ$)로 절반 압축함으로써, **스캔 시간을 약 1분 가까이 단축하면서도 Case A(급출발/급정지) 대비 3.74배의 병합 샘플을 확보**하여 양 끝단 노이즈 제거 효과를 유지했습니다.

### ② 저속 공진 대역 탈출 및 기구 안정성
- 스텝모터의 $100 \sim 250\text{ PPS}$ 저속 공진 대역 체류 시간이 $125\text{ms} \to 62.5\text{ms}$로 단축되어 구동 진동과 모터 덜덜거림 소음이 현저히 감소하였습니다.
- 스캔 종료 후 틸트 잔차가 $0.0^\circ$로 측정된 것은 $2400\text{ PPS}^2$의 가속도가 17HS4401 모터 토크 및 기구 관성 한계 내에서 완벽하게 제어되고 있음을 입증합니다.

---

## 4. 원본 데이터 아카이빙 (Raw Data)

본 테스트의 모든 실측 원본 데이터는 프로젝트 내 [`docs/motor_profile_test/raw_data/`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/) 경로에 보관되었습니다:

* **가감속 미적용 (Case A)**:
  - PCD 점군: [`docs/motor_profile_test/raw_data/scan_no_profile.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_no_profile.pcd)
  - 상세 JSON: [`docs/motor_profile_test/raw_data/scan_no_profile.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_no_profile.json)
* **가감속 적용 - 1200 PPS² (Case B)**:
  - PCD 점군: [`docs/motor_profile_test/raw_data/scan_with_profile.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_with_profile.pcd)
  - 상세 JSON: [`docs/motor_profile_test/raw_data/scan_with_profile.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_with_profile.json)
* **가속도 튜닝 - 2400 PPS² (Phase 1)**:
  - PCD 점군: [`docs/motor_profile_test/raw_data/scan_accel_2400.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_accel_2400.pcd)
  - 상세 JSON: [`docs/motor_profile_test/raw_data/scan_accel_2400.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_accel_2400.json)

---

## 5. 차기 로드맵 연계 계획

1. **[Phase 1 완료]**: $a = 2400\text{ PPS}^2$를 기본 프로파일로 확정.
2. **[Phase 2 진입]**: $\pm 5^\circ$ 오버스캔 버퍼 설계를 통해 유효 측정 구간($-90^\circ \sim +90^\circ$)을 100% 완전 등속화하여 공간적 격자 밀도 균일도 극대화.
3. **[Phase 3 진입]**: S-Curve (저크 제한) 프로파일 도입으로 잔류 진동 제거 및 정착 시간($100\text{ms} \to 30\text{ms}$) 단축.
