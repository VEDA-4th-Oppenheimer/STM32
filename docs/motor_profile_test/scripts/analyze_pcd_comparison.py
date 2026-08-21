#!/usr/bin/env python3
# ============================================================================
#  analyze_pcd_comparison.py -- PCD 3D 포인트 클라우드 정량 비교 분석기
# ============================================================================
import numpy as np
import json
import sys
import os

def load_pcd(filepath):
    """ASCII PCD 파일을 로드하여 (N, 3) numpy 배열로 반환합니다."""
    points = []
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
                    # 유효한 점만 수집 (0,0,0 결측 제외)
                    if not (x == 0.0 and y == 0.0 and z == 0.0) and not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append([x, y, z])
                except ValueError:
                    continue
    return np.array(points)

def fit_plane_ransac(points, dist_threshold=0.03, max_iters=500):
    """RANSAC으로 주 평면(벽면/바닥)을 찾고 인라이어 잔차(RMSE)를 계산합니다."""
    if len(points) < 100:
        return 0.0, 0, None
    
    best_inliers = []
    best_plane = None
    N = len(points)
    
    for _ in range(max_iters):
        # 3점 랜덤 선택
        idx = np.random.choice(N, 3, replace=False)
        p1, p2, p3 = points[idx]
        
        # 법선 벡터
        v1 = p2 - p1
        v2 = p3 - p1
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        d = -np.dot(normal, p1)
        
        # 모든 점과의 수직 거리
        dists = np.abs(np.dot(points, normal) + d)
        inliers = np.where(dists < dist_threshold)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane = (normal, d)
            
    if len(best_inliers) == 0:
        return 0.0, 0, None
        
    inlier_pts = points[best_inliers]
    normal, d = best_plane
    # 정밀 평면 재피팅 (SVD)
    centroid = np.mean(inlier_pts, axis=0)
    shifted = inlier_pts - centroid
    _, _, vh = np.linalg.svd(shifted)
    ref_normal = vh[2, :]
    ref_d = -np.dot(ref_normal, centroid)
    
    residuals = np.abs(np.dot(inlier_pts, ref_normal) + ref_d)
    rmse = np.sqrt(np.mean(residuals**2))
    std_dev = np.std(residuals)
    
    return float(rmse), len(best_inliers), {
        "rmse_mm": float(rmse * 1000.0),
        "std_mm": float(std_dev * 1000.0),
        "max_err_mm": float(np.max(residuals) * 1000.0),
        "inlier_ratio_pct": float(len(best_inliers) / N * 100.0)
    }

def calculate_smoothness_jitter(points):
    """인접 점들 간의 2차 차분을 통한 표면 거칠기(Jitter / Roughness) 지표 계산"""
    if len(points) < 5:
        return 0.0
    # 2차 차분: d2 = p[i+1] - 2*p[i] + p[i-1]
    diff2 = points[2:] - 2*points[1:-1] + points[:-2]
    jitter_norm = np.linalg.norm(diff2, axis=1)
    mean_jitter_mm = float(np.mean(jitter_norm) * 1000.0)
    std_jitter_mm = float(np.std(jitter_norm) * 1000.0)
    return {
        "mean_jitter_mm": mean_jitter_mm,
        "std_jitter_mm": std_jitter_mm
    }

def analyze(pcd_no_profile, pcd_with_profile):
    pts_no = load_pcd(pcd_no_profile)
    pts_with = load_pcd(pcd_with_profile)
    
    np.random.seed(42)  # 재현성
    _, _, plane_no = fit_plane_ransac(pts_no)
    np.random.seed(42)
    _, _, plane_with = fit_plane_ransac(pts_with)
    
    jit_no = calculate_smoothness_jitter(pts_no)
    jit_with = calculate_smoothness_jitter(pts_with)
    
    result = {
        "case_a_no_profile": {
            "total_valid_points": len(pts_no),
            "plane_residuals": plane_no,
            "jitter": jit_no
        },
        "case_b_with_profile": {
            "total_valid_points": len(pts_with),
            "plane_residuals": plane_with,
            "jitter": jit_with
        }
    }
    
    # 개선율 계산
    if plane_no and plane_with:
        rmse_imprv = (plane_no["rmse_mm"] - plane_with["rmse_mm"]) / plane_no["rmse_mm"] * 100.0
        jit_imprv = (jit_no["mean_jitter_mm"] - jit_with["mean_jitter_mm"]) / jit_no["mean_jitter_mm"] * 100.0
        result["improvement"] = {
            "plane_rmse_improvement_pct": rmse_imprv,
            "jitter_improvement_pct": jit_imprv
        }
        
    return result

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 analyze_pcd_comparison.py <no_profile.pcd> <with_profile.pcd>")
        sys.exit(1)
        
    res = analyze(sys.argv[1], sys.argv[2])
    print(json.dumps(res, indent=2))
