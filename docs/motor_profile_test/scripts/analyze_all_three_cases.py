#!/usr/bin/env python3
import json
import numpy as np

def parse_json(path):
    with open(path) as f:
        d = json.load(f)
    s = d.get("scan", {})
    diag = d.get("diagnostics", {})
    h = d.get("mechanism", {}).get("home", {})
    t0, t1 = s.get("started_at_ns", 0), s.get("ended_at_ns", 0)
    dur = (t1 - t0)/1e9 if t1 > t0 else 0
    tot = s.get("sample_count", 0)
    val = s.get("valid_count", 0)
    return {
        "session_id": d.get("session_id", ""),
        "total_cells": tot,
        "valid_points": val,
        "missing_cells": tot - val,
        "fill_rate_pct": round(val*100.0/tot, 2) if tot else 0,
        "duration_sec": round(dur, 1),
        "duration_min": round(dur/60.0, 2),
        "merged_samples": diag.get("merged_sample_count", 0),
        "home_tilt_err_deg": h.get("tilt_ddeg", 0)/10.0,
        "home_pan_err_deg": h.get("pan_ddeg", 0)/10.0,
    }

def analyze_pcd(filepath, max_points=10000):
    pts = []
    total = 0
    with open(filepath, 'r') as f:
        header = True
        for line in f:
            if header:
                if line.startswith("DATA"):
                    header = False
                continue
            parts = line.strip().split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    if (x != 0.0 or y != 0.0 or z != 0.0) and not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        total += 1
                        pts.append((x, y, z))
                except ValueError:
                    continue
    arr = np.array(pts, dtype=np.float32)
    if len(arr) > max_points:
        step = len(arr) // max_points
        sampled = arr[::step]
    else:
        sampled = arr

    centroid = np.mean(sampled, axis=0)
    shifted = sampled - centroid
    _, _, vh = np.linalg.svd(shifted, full_matrices=False)
    normal = vh[2, :]
    dists = np.abs(np.dot(shifted, normal)) * 1000.0  # mm
    sorted_dists = np.sort(dists)
    inliers = sorted_dists[:int(len(sorted_dists) * 0.9)]
    rmse_mm = float(np.sqrt(np.mean(inliers**2)))
    
    diff2 = sampled[2:] - 2*sampled[1:-1] + sampled[:-2]
    jitter = np.linalg.norm(diff2, axis=1) * 1000.0
    jitter_inliers = np.sort(jitter)[:int(len(jitter) * 0.9)]
    mean_jitter_mm = float(np.mean(jitter_inliers))
    
    return {
        "valid_points_in_pcd": total,
        "plane_rmse_mm": round(rmse_mm, 2),
        "surface_jitter_mm": round(mean_jitter_mm, 2)
    }

def main():
    cases = [
        ("Case A (가감속 OFF)", "/tmp/scan_no_profile.json", "/tmp/scan_no_profile.pcd"),
        ("Case B (가감속 1200 PPS2)", "/tmp/scan_with_profile.json", "/tmp/scan_with_profile.pcd"),
        ("Phase 1 (튜닝 2400 PPS2)", "/tmp/scan_accel_2400.json", "/tmp/scan_accel_2400.pcd"),
    ]
    
    results = {}
    for name, jpath, ppath in cases:
        meta = parse_json(jpath)
        geom = analyze_pcd(ppath)
        meta.update(geom)
        results[name] = meta
        
    print(json.dumps(results, indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
