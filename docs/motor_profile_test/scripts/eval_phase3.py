#!/usr/bin/env python3
import json
import re
import math
import numpy as np

def analyze_dataset(name, pcd_path, json_path):
    print(f"=== {name} ===")
    # 1. JSON parsing
    with open(json_path, 'r', encoding='utf-8') as f:
        meta = json.load(f)
    
    total_cells = meta.get("summary", {}).get("total_cells", 40400)
    valid_cells = meta.get("summary", {}).get("valid_cells", 0)
    missing_cells = meta.get("summary", {}).get("missing_cells", 0)
    merged_samples = meta.get("summary", {}).get("merged_samples", 0)
    fill_rate = (valid_cells / total_cells) * 100.0
    
    # 2. PCD parsing
    pcd_points = 0
    pcd_valid = 0
    coords = []
    with open(pcd_path, 'r', encoding='utf-8') as f:
        for line in f:
            if line.startswith("POINTS"):
                pcd_points = int(line.split()[1])
            elif not line.startswith("#") and not line.startswith("VERSION") and not line.startswith("FIELDS") \
                and not line.startswith("SIZE") and not line.startswith("TYPE") and not line.startswith("COUNT") \
                and not line.startswith("WIDTH") and not line.startswith("HEIGHT") and not line.startswith("VIEWPOINT") \
                and not line.startswith("DATA"):
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                            coords.append([x, y, z])
                            pcd_valid += 1
                    except ValueError:
                        pass
    
    coords = np.array(coords)
    print(f"Total Cells: {total_cells}")
    print(f"Valid Cells: {valid_cells} (PCD valid: {pcd_valid})")
    print(f"Missing Cells: {missing_cells}")
    print(f"Fill Rate: {fill_rate:.2f}%")
    print(f"Merged Samples (multi-sample cells): {merged_samples}")
    
    # Scan duration from daemon / timestamps
    start_ns = meta.get("metadata", {}).get("scan_start_ns", 0)
    # Check if point timestamps exist in json
    grid = meta.get("grid", [])
    t_min, t_max = None, None
    for row in grid:
        for cell in row:
            if cell and cell.get("t"):
                t = cell["t"]
                if t_min is None or t < t_min: t_min = t
                if t_max is None or t > t_max: t_max = t
    
    duration_s = (t_max - t_min) if (t_min and t_max) else 0
    print(f"Scan Duration (from points): {duration_s:.1f} s ({duration_s/60:.2f} min)")
    
    # Calculate spatial noise (surface roughness / std dev of distance)
    if len(coords) > 0:
        dists = np.linalg.norm(coords, axis=1)
        print(f"Distance stats: mean={np.mean(dists):.3f}m, std={np.std(dists):.3f}m, min={np.min(dists):.3f}m, max={np.max(dists):.3f}m")
    
    return {
        "name": name,
        "valid_cells": valid_cells,
        "missing_cells": missing_cells,
        "fill_rate": fill_rate,
        "merged_samples": merged_samples,
        "duration_s": duration_s,
    }

if __name__ == "__main__":
    r1 = analyze_dataset("Case A (급출발 No Profile)", "docs/motor_profile_test/raw_data/scan_no_profile.pcd", "docs/motor_profile_test/raw_data/scan_no_profile.json")
    print()
    r2 = analyze_dataset("Case B (사다리꼴 1200 PPS²)", "docs/motor_profile_test/raw_data/scan_with_profile.pcd", "docs/motor_profile_test/raw_data/scan_with_profile.json")
    print()
    r3 = analyze_dataset("Phase 1 (사다리꼴 2400 PPS²)", "docs/motor_profile_test/raw_data/scan_accel_2400.pcd", "docs/motor_profile_test/raw_data/scan_accel_2400.json")
    print()
    r4 = analyze_dataset("Phase 3 (S-Curve 2400 PPS²)", "docs/motor_profile_test/raw_data/scan_scurve_phase3.pcd", "docs/motor_profile_test/raw_data/scan_scurve_phase3.json")
