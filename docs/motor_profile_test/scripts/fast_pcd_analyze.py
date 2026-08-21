#!/usr/bin/env python3
# ============================================================================
#  fast_pcd_analyze.py -- 초경량 메모리 절약형 3D PCD 비교 분석기
# ============================================================================
import numpy as np
import json
import sys

def load_and_sample_pcd(filepath, max_points=5000):
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
    # 고른 샘플링
    arr = np.array(pts, dtype=np.float32)
    if len(arr) > max_points:
        step = len(arr) // max_points
        sampled = arr[::step]
    else:
        sampled = arr
    return sampled, total

def analyze_dataset(pts, total_count):
    # 1. 분산 및 표준편차 (전체 형상 안정성)
    std_xyz = np.std(pts, axis=0) * 1000.0  # mm
    
    # 2. 중심점 및 주축 피팅 (SVD 평면 잔차)
    centroid = np.mean(pts, axis=0)
    shifted = pts - centroid
    _, s, vh = np.linalg.svd(shifted, full_matrices=False)
    normal = vh[2, :]  # 주 평면 법선
    dists = np.abs(np.dot(shifted, normal)) * 1000.0  # mm
    
    # 상위 10% 이상치 제외한 잔차 (안정 영역)
    sorted_dists = np.sort(dists)
    inliers = sorted_dists[:int(len(sorted_dists) * 0.9)]
    rmse_mm = float(np.sqrt(np.mean(inliers**2)))
    std_mm = float(np.std(inliers))
    max_mm = float(np.max(inliers))
    
    # 3. 2차 차분 지터 (인접 점 간 거칠기)
    diff2 = pts[2:] - 2*pts[1:-1] + pts[:-2]
    jitter = np.linalg.norm(diff2, axis=1) * 1000.0
    jitter_inliers = np.sort(jitter)[:int(len(jitter) * 0.9)]
    mean_jitter_mm = float(np.mean(jitter_inliers))
    
    return {
        "total_points": total_count,
        "sampled_points": len(pts),
        "plane_rmse_mm": round(rmse_mm, 2),
        "plane_std_mm": round(std_mm, 2),
        "plane_max_err_mm": round(max_mm, 2),
        "mean_jitter_mm": round(mean_jitter_mm, 2),
        "spread_std_xyz_mm": [round(float(v), 1) for v in std_xyz]
    }

def main(pcd_no, pcd_with):
    pts_no, tot_no = load_and_sample_pcd(pcd_no)
    pts_with, tot_with = load_and_sample_pcd(pcd_with)
    
    res_no = analyze_dataset(pts_no, tot_no)
    res_with = analyze_dataset(pts_with, tot_with)
    
    rmse_diff = res_no["plane_rmse_mm"] - res_with["plane_rmse_mm"]
    rmse_pct = (rmse_diff / res_no["plane_rmse_mm"]) * 100.0 if res_no["plane_rmse_mm"] > 0 else 0
    
    jit_diff = res_no["mean_jitter_mm"] - res_with["mean_jitter_mm"]
    jit_pct = (jit_diff / res_no["mean_jitter_mm"]) * 100.0 if res_no["mean_jitter_mm"] > 0 else 0
    
    report = {
        "case_a_no_profile": res_no,
        "case_b_with_profile": res_with,
        "comparison": {
            "plane_rmse_reduction_mm": round(rmse_diff, 2),
            "plane_rmse_improvement_pct": round(rmse_pct, 1),
            "surface_jitter_reduction_mm": round(jit_diff, 2),
            "surface_jitter_improvement_pct": round(jit_pct, 1)
        }
    }
    print(json.dumps(report, indent=2))

if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit(1)
    main(sys.argv[1], sys.argv[2])
