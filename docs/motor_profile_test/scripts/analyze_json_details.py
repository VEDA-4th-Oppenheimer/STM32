#!/usr/bin/env python3
import json

def analyze_json(no_path, with_path):
    with open(no_path) as f:
        j_no = json.load(f)
    with open(with_path) as f:
        j_with = json.load(f)
        
    pts_no = j_no.get("points", [])
    pts_with = j_with.get("points", [])
    
    # 틸트 각도 범위 및 시간 계산
    t0_no = pts_no[0].get("time_ns", 0) if pts_no else 0
    t1_no = pts_no[-1].get("time_ns", 0) if pts_no else 0
    dur_no_s = (t1_no - t0_no) / 1e9
    
    t0_with = pts_with[0].get("time_ns", 0) if pts_with else 0
    t1_with = pts_with[-1].get("time_ns", 0) if pts_with else 0
    dur_with_s = (t1_with - t0_with) / 1e9
    
    print(json.dumps({
        "case_a_no_profile": {
            "point_count": len(pts_no),
            "scan_duration_sec": round(dur_no_s, 2),
            "session_id": j_no.get("session_id", ""),
            "scan_id": j_no.get("scan_id", "")
        },
        "case_b_with_profile": {
            "point_count": len(pts_with),
            "scan_duration_sec": round(dur_with_s, 2),
            "session_id": j_with.get("session_id", ""),
            "scan_id": j_with.get("scan_id", "")
        }
    }, indent=2))

if __name__ == "__main__":
    analyze_json("/tmp/scan_no_profile.json", "/tmp/scan_with_profile.json")
