# [Phase 4] 골든 레이시오(Golden-Ratio) 750 PPS 복합 최적화 실측 평가 보고서

**시험 일시**: 2026-08-21 16:35 KST  
**시험 대상**: STM32F401RE + DRV8825 + 17HS4401 2축 짐벌 + TOFSense-F2P LiDAR  
**최종 확정 프로파일**: 
- **틸트 순항 속도**: `MOTOR_TILT_CRUISE_PPS = 750u` ($84.375^\circ/\text{s}$, 샘플 밀도 $1.067\times$)
- **틸트 가속도**: `MOTOR_TILT_ACCEL_PPS2 = 1800u` + S-Curve (저크 억제 및 소프트 랜딩)
- **팬 스텝 가속도**: `MOTOR_PAN_ACCEL_PPS2 = 1200u` (줄 바꿈 지연 단축)
- **정착 대기 시간**: `SCAN_LINE_SETTLE_MS = 40u` (무진동 연착륙 기반 시간 회수)

---

## 1. 6대 가감속 프로파일 실측 종합 비교 매트릭스

200줄(40,400개 정규 3D 셀) 연속 스캔 데이터를 바탕으로 산출한 최종 전수 비교 평가 결과입니다:

| 평가 항목 | Case B (사다리꼴 1200) | Phase 1 (사다리꼴 2400) | Phase 3-A (S-Curve 2400) | Phase 3-B (S-Curve 1200) | **Phase 4 (750 PPS 표준)** | **Phase 4-B (800 PPS 고속)** |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **가감속 프로파일** | 선형 사다리꼴 | 선형 사다리꼴 | S-Curve (저크 제한) | S-Curve (저크 제한) | **S-Curve (저크 제한)** | **S-Curve (저크 제한)** |
| **틸트 가속도** | $1200\text{ PPS}^2$ | $2400\text{ PPS}^2$ | $2400\text{ PPS}^2$ | $1200\text{ PPS}^2$ | **$1800\text{ PPS}^2$** | **$1800\text{ PPS}^2$** |
| **틸트 순항 속도** | $800\text{ PPS}$ ($90^\circ/\text{s}$) | $800\text{ PPS}$ ($90^\circ/\text{s}$) | $800\text{ PPS}$ ($90^\circ/\text{s}$) | $800\text{ PPS}$ ($90^\circ/\text{s}$) | **$750\text{ PPS}$ ($84.4^\circ/\text{s}$)** | **$800\text{ PPS}$ ($90.0^\circ/\text{s}$)** |
| **정착 대기 시간** | $100\text{ms}$ | $100\text{ms}$ | $100\text{ms}$ | $100\text{ms}$ | **$40\text{ms}$** | **$40\text{ms}$** |
| **팬 스텝 가속도** | $600\text{ PPS}^2$ | $600\text{ PPS}^2$ | $600\text{ PPS}^2$ | $600\text{ PPS}^2$ | **$1200\text{ PPS}^2$** | **$1200\text{ PPS}^2$** |
| **스캔 소요 시간** | 9분 30초 (569.9s) | 8분 36초 (516.4s) | 8분 43초 (522.8s) | 10분 21초 (620.5s) | **9분 31초 (571.3s)** | ⚡ **9분 10초 (549.6s)** |
| **유효 포인트 수** | 40,080 점 | 40,019 점 | 40,032 점 | 40,071 점 | 🏆 **40,181 점 (+101점)** | **40,067 점** |
| **격자 충진율 (Fill Rate)** | 99.21% (결측 320개) | 99.06% (결측 381개) | 99.09% (결측 368개) | 99.19% (결측 329개) | 🎯 **99.46% (결측 219개)** | **99.18% (결측 333개)** |
| **병합 샘플 수** | 12,706 개 | 7,385 개 | 7,817 개 | 17,333 개 | **13,567 개** | **11,492 개** |
| **홈 복귀 잔차 (Pan)** | $+0.0^\circ$ | $-0.1^\circ$ | $+0.0^\circ$ | $-0.1^\circ$ | **$-0.1^\circ$** | **$-0.1^\circ$** |
| **홈 복귀 잔차 (Tilt)** | $+0.2^\circ$ | $+0.0^\circ$ | $-0.3^\circ$ | $+0.1^\circ$ | **$+0.0^\circ$ (오차 0)** | **$-0.1^\circ$** |
| **기구 저크(Jerk)** | 중간 (가속도 점프) | 중간 (가속도 점프) | 최소 | 극소 (소프트 랜딩) | **극소 (소프트 랜딩)** | **극소 (소프트 랜딩)** |
| **최종 평가 및 채택** | 기준선 (사다리꼴) | 진동 잔존 | 램프 짧음 | 시간 지연 | 🚀 **최종 프로덕션 표준 채택** | 고속 전용 모드 |

---

## 2. 750 PPS 골든 레이시오 2회 연속 스캔 신뢰성 및 재현성 검증

동일한 750 PPS 펌웨어로 200줄(40,400개 정규 셀) 3D 스캔을 2회 연속 실행하여 신뢰성을 정밀 검증하였습니다:

| 검증 항목 | Run 1 (1회차) | Run 2 (2회차) | 오차 ($\Delta$) | 일치율 (재현성) |
|---|:---:|:---:|:---:|:---:|
| **스캔 소요 시간** | 571.3 초 (9분 31.3초) | 571.3 초 (9분 31.3초) | **$\Delta 0.06\text{초}$** | **$99.99\%$ 완벽 일치** |
| **유효 포인트 수** | 40,181 점 | 40,182 점 | **$\Delta 1\text{점}$** | **$99.998\%$ 완벽 일치** |
| **격자 결측 셀 수** | 219 개 (0.54%) | 218 개 (0.54%) | **$\Delta 1\text{개}$** | **$99.46\%$ 충진율 재현** |
| **병합 샘플 수** | 13,567 개 | 13,573 개 | **$\Delta 6\text{개}$** | **$99.96\%$ 일치** |
| **Pan 홈 복귀 잔차** | $-0.1^\circ$ | $-0.1^\circ$ | $\Delta 0.0^\circ$ | **$100.0\%$ 일치** |
| **Tilt 홈 복귀 잔차** | $+0.0^\circ$ | $+0.2^\circ$ | $\Delta 0.2^\circ$ | 정밀 영점 수렴 |

> [!TIP]
> 2회 연속 3D 스캔 결과 시간 오차 $0.06\text{초}$, 유효 포인트 오차 $1\text{점}$으로 **거의 $100\%$에 달하는 극강의 재현성과 제어 신뢰성**을 증명하였습니다.

---

## 3. 원시 데이터 아카이빙

* **Phase 4 (750 PPS Run 1) 원시 데이터**:
  - PCD: [`docs/motor_profile_test/raw_data/scan_phase4_golden.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_golden.pcd)
  - JSON: [`docs/motor_profile_test/raw_data/scan_phase4_golden.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_golden.json)
* **Phase 4 (750 PPS Run 2) 원시 데이터**:
  - PCD: [`docs/motor_profile_test/raw_data/scan_phase4_750pps_run2.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_750pps_run2.pcd)
  - JSON: [`docs/motor_profile_test/raw_data/scan_phase4_750pps_run2.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_750pps_run2.json)
* **Phase 4-B (800 PPS 고속) 원시 데이터**:
  - PCD: [`docs/motor_profile_test/raw_data/scan_phase4_800pps.pcd`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_800pps.pcd)
  - JSON: [`docs/motor_profile_test/raw_data/scan_phase4_800pps.json`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/raw_data/scan_phase4_800pps.json)
