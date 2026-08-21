# 2축 짐벌 모터 가감속 프로파일 개선 3단계 로드맵 계획서

**작성일**: 2026-08-21  
**문서 위치**: `docs/motor_profile_test/motor_profile_roadmap_plan.md`  
**대상 시스템**: STM32F401RE + DRV8825 + 17HS4401 (Tilt/Pan) + TOFSense-F2P LiDAR + MT6701 엔코더  

---

## 1. 개요 및 배경

[모터 가감속 실측 비교 보고서](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_test_report.md)에 따르면, 사다리꼴 가감속 프로파일(Case B) 도입으로 끝단 결측이 19.8% 감소하고 노이즈 필터링 밀도가 6.4배 향상되는 성과를 확인하였습니다.

그러나 다음의 기술적 개선 과제가 남아 있습니다:
1. **스캔 시간 트레이드오프**: 보수적인 가속도($1200\text{ PPS}^2$)로 인해 스캔 소요 시간이 7분 35초 $\to$ 9분 30초로 +25.3% 증가.
2. **공간적 점 밀도 불균일**: 가감속 구간($\pm 60^\circ \sim \pm 90^\circ$)과 등속 순항 구간($-60^\circ \sim +60^\circ$)의 샘플 밀도 편차.
3. **가속도 불연속에 따른 저크(Jerk)**: 사다리꼴 모서리 구간에서의 미세 기구 충격.

이를 해결하기 위해 **3단계 단계적 개선 로드맵**을 수립하고 순차 검증을 진행합니다.

---

## 2. 3단계 개선 로드맵

```mermaid
graph TD
    A[Phase 1: 가속도 튜닝 (1200 → 2400 PPS²)] -->|검증 완료| B[Phase 2: 오버스캔 100% 등속 버퍼]
    B -->|검증 완료| C[Phase 3: S-Curve / Cosine 저크 제한 프로파일]
    
    style A fill:#e1f5fe,stroke:#0288d1,stroke-width:2px
    style B fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    style C fill:#e8f5e9,stroke:#388e3c,stroke-width:2px
```

### 📍 [Phase 1] 가속도 튜닝 (✅ 완료)
* **결과**: $a = 2400\text{ PPS}^2$ 적용으로 **스캔 시간 53.5초 단축 (9분 30초 $\to$ 8분 36초)**, 무탈조(오차 $0.0^\circ$) 및 고품질 노이즈 필터링(병합 샘플 7,385개) 검증 완료. (상세 보고서: [`motor_profile_phase1_tuning_report.md`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_phase1_tuning_report.md))

---

### 📍 [Phase 2] 오버스캔(Over-scan) 등속 버퍼 실측 및 물리 한계 분석 (⚠️ 검증 완료)
* **실측 검증 결과**:
  - 짐벌 틸트부의 물리적 최대 가동 범위가 정확히 $\pm 90.0^\circ$로 하드웨어 스토퍼/브라켓 한계가 존재함.
  - $\pm 95.0^\circ$ 오버스캔 시도 시 $+90.0^\circ$ 지점에서 기구적 간섭이 발생하여 모터가 멈추고, 엔코더와의 $5.0^\circ$ 차이를 펌웨어가 정상 감지하여 `ERR_STALL (code=5, axis=2)`을 즉시 발생시킴 (탈조 안전 메커니즘 정상 동작 확인).
* **설계 결론**:
  - $\pm 90.0^\circ$ 풀스팬 스캔 시에는 하드웨어 간섭 방지를 위해 스윕 범위를 $\pm 90.0^\circ$로 유지하고, **[Phase 3] S-Curve 저크 제한 프로파일**을 통해 감속 전환부의 진동을 억제하는 것이 올바른 최적화 방향임.

---

### 📍 [Phase 3] S-Curve (저크 제한) 가감속 프로파일 (✅ 완료)
* **결과**: 2차 포물선 벨형 가속도 변조(Floor 25% ~ Peak 100%) 적용으로 **스캔 시간 47.1초 단축 (9분 30초 $\to$ 8분 43초)**, **기구 저크 충격 제거**, **충진율 99.09%** 및 **무탈조 정밀 수렴 (Pan 0.0°, Tilt -0.3°)** 달성. (상세 보고서: [`motor_profile_phase3_scurve_report.md`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_phase3_scurve_report.md))

---

## 3. 로드맵 최종 산출물 및 보고서 체계

| 단계 | 산출물 보고서 | 원시 데이터 (PCD / JSON) | 상태 |
|---|---|---|:---:|
| **기준선 (Baseline)** | [`motor_profile_test_report.md`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_test_report.md) | `scan_no_profile.*`, `scan_with_profile.*` | ✅ 완료 |
| **Phase 1 (가속도 2400)** | [`motor_profile_phase1_tuning_report.md`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_phase1_tuning_report.md) | `scan_accel_2400.*` | ✅ 완료 |
| **Phase 2 (오버스캔 분석)** | 본 로드맵 계획서 2절 요약 | 기구적 한계 검증 완료 | ⚠️ 완료 |
| **Phase 3 (S-Curve 채택)** | [`motor_profile_phase3_scurve_report.md`](file:///c:/VEDA_Workspace/CLionProject/docs/motor_profile_test/motor_profile_phase3_scurve_report.md) | `scan_scurve_phase3.*` | 🚀 **기본 채택** |
