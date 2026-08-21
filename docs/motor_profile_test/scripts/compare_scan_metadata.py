#!/usr/bin/env python3
import json
import sys

def parse_scan_json(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)
        
    scan_meta = data.get("scan", {})
    diag = data.get("diagnostics", {})
    home = data.get("mechanism", {}).get("home", {})
    
    t0 = scan_meta.get("started_at_ns", 0)
    t1 = scan_meta.get("ended_at_ns", 0)
    dur_sec = (t1 - t0) / 1e9 if t1 > t0 else 0
    
    measurements = data.get("measurements", [])
    spreads = [m.get("spread_mm", 0) for m in measurements if m.get("valid", False)]
    avg_spread = sum(spreads) / len(spreads) if spreads else 0
    
    return {
        "session_id": data.get("session_id", ""),
        "total_cells": scan_meta.get("sample_count", 0),
        "valid_points": scan_meta.get("valid_count", 0),
        "fill_rate_pct": round((scan_meta.get("valid_count", 0) / scan_meta.get("sample_count", 1)) * 100, 2),
        "duration_sec": round(dur_sec, 2),
        "duration_min": round(dur_sec / 60.0, 2),
        "home_tilt_err_ddeg": home.get("tilt_ddeg", 0),
        "home_pan_err_ddeg": home.get("pan_ddeg", 0),
        "avg_cell_spread_mm": round(avg_spread, 2),
        "merged_samples": diag.get("merged_sample_count", 0),
        "refused_cells": diag.get("avg_refused_cell_count", 0)
    }

def main():
    path_no = sys.argv[1] if len(sys.argv) > 1 else "/tmp/scan_no_profile.json"
    path_with = sys.argv[2] if len(sys.argv) > 2 else "/tmp/scan_with_profile.json"
    
    res_no = parse_scan_json(path_no)
    res_with = parse_scan_json(path_with)
    
    out = {
        "case_a_no_profile": res_no,
        "case_b_with_profile": res_with
    }
    print(json.dumps(out, indent=2))

if __name__ == "__main__":
    main()
