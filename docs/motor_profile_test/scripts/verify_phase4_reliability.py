import json
import os

path1 = "/tmp/scan_phase4_golden.json" if os.path.exists("/tmp/scan_phase4_golden.json") else "docs/motor_profile_test/raw_data/scan_phase4_golden.json"
path2 = "/tmp/scan_phase4_750pps_run2.json" if os.path.exists("/tmp/scan_phase4_750pps_run2.json") else "docs/motor_profile_test/raw_data/scan_phase4_750pps_run2.json"

with open(path1, "r", encoding="utf-8") as f:
    d1 = json.load(f)
with open(path2, "r", encoding="utf-8") as f:
    d2 = json.load(f)

s1, s2 = d1["scan"], d2["scan"]
v1, v2 = s1["valid_count"], s2["valid_count"]
t1 = (s1["ended_at_ns"] - s1["started_at_ns"]) / 1e9
t2 = (s2["ended_at_ns"] - s2["started_at_ns"]) / 1e9
m1 = d1.get("diagnostics", {}).get("merged_sample_count", 0)
m2 = d2.get("diagnostics", {}).get("merged_sample_count", 0)
h1 = d1.get("mechanism", {}).get("home", {})
h2 = d2.get("mechanism", {}).get("home", {})

print("================================================================")
print("        [Phase 4: 750 PPS 골든 레이시오] 2회 연속 신뢰성/재현성 검증")
print("================================================================")
print(f"1. 스캔 소요 시간 : Run 1 = {t1:.1f}초 vs Run 2 = {t2:.1f}초  (Δ {abs(t1-t2):.2f}초, {(1-abs(t1-t2)/t1)*100:.2f}% 일치)")
print(f"2. 유효 포인트 수 : Run 1 = {v1:,}점 vs Run 2 = {v2:,}점  (Δ {abs(v1-v2)}점, {(1-abs(v1-v2)/v1)*100:.3f}% 일치)")
print(f"3. 격자 결측 셀 수: Run 1 = {40400-v1}개 vs Run 2 = {40400-v2}개 (Δ {abs((40400-v1)-(40400-v2))}개)")
print(f"4. 격자 충진율    : Run 1 = {v1/404.0:.2f}% vs Run 2 = {v2/404.0:.2f}%")
print(f"5. 병합 샘플 수   : Run 1 = {m1:,}개 vs Run 2 = {m2:,}개  (Δ {abs(m1-m2)}개, {(1-abs(m1-m2)/m1)*100:.2f}% 일치)")
print(f"6. Pan 홈 복귀오차: Run 1 = {h1.get('pan_ddeg',0)/10.0:+.1f}° vs Run 2 = {h2.get('pan_ddeg',0)/10.0:+.1f}°")
print(f"7. Tilt 홈 복귀오차: Run 1 = {h1.get('tilt_ddeg',0)/10.0:+.1f}° vs Run 2 = {h2.get('tilt_ddeg',0)/10.0:+.1f}°")
print("================================================================")
