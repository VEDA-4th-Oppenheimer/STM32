#!/usr/bin/env python3
import time
import json
import urllib.request
import subprocess
import os
import sys
from pathlib import Path

RPI_HOST = "pi@172.20.26.191"
REPO_ROOT = Path(__file__).resolve().parent.parent.parent
RAW_DATA_DIR = REPO_ROOT / "docs" / "motor_profile_test" / "raw_data"

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

def wait_and_collect():
    log("Phase 1 (2400 PPS2) 3D 스캔 진행 모니터링 시작...")
    start_time = time.time()
    last_pts = 0
    
    while True:
        try:
            req = urllib.request.urlopen("http://172.20.26.191:8080/api/state", timeout=5)
            data = json.loads(req.read().decode())
            daemon_st = data.get("topics", {}).get("adts/state/daemon", {})
            scanning = daemon_st.get("scanning", False)
            prog = data.get("topics", {}).get("adts/event/progress", {})
            pts = prog.get("points", 0)
            pct = prog.get("percent", 0)
            cur_pan = daemon_st.get("cur_pan_ddeg", 0) / 10.0
            cur_tilt = daemon_st.get("cur_tilt_ddeg", 0) / 10.0
            
            elapsed = time.time() - start_time
            if pts != last_pts or int(elapsed) % 15 == 0:
                log(f"진행: {pct:3d}% ({pts:5d}/40200 pts) | Pan: {cur_pan:5.1f}°, Tilt: {cur_tilt:5.1f}° | 경과: {int(elapsed)}초")
                last_pts = pts
                
            if not scanning and pts >= 35000:
                log(f"★ 3D 스캔 완료 감지! 총 소요 시간: {elapsed:.1f}초 ({elapsed/60.0:.2f}분)")
                break
        except Exception as e:
            log(f"상태 폴링 대기 중... ({e})")
            
        time.sleep(5)

    # 잠시 디스크 flush 대기
    time.sleep(3)
    
    # RPi 에서 최신 생성된 pcd, json 파일명 조회
    log("RPi 에서 최신 스캔 파일 검색 중...")
    ssh_cmd = ["ssh", RPI_HOST, "sudo ls -t /var/lib/adts/scans/*.pcd | head -n 1"]
    res = subprocess.run(ssh_cmd, capture_output=True, text=True, check=True)
    latest_pcd = res.stdout.strip()
    latest_json = latest_pcd.replace(".pcd", "_pan_tilt_lidar.json")
    
    log(f"발견된 최신 파일:\n  PCD : {latest_pcd}\n  JSON: {latest_json}")
    
    # RPi 에서 /tmp 로 복사 및 권한 해제
    prep_cmd = f"echo 1234 | sudo -S cp {latest_pcd} /tmp/scan_accel_2400.pcd && " \
               f"echo 1234 | sudo -S cp {latest_json} /tmp/scan_accel_2400.json && " \
               f"echo 1234 | sudo -S chmod 666 /tmp/scan_accel_2400.*"
    subprocess.run(["ssh", RPI_HOST, prep_cmd], check=True)
    
    # 로컬 raw_data 로 SCP 다운로드
    RAW_DATA_DIR.mkdir(parents=True, exist_ok=True)
    local_pcd = RAW_DATA_DIR / "scan_accel_2400.pcd"
    local_json = RAW_DATA_DIR / "scan_accel_2400.json"
    
    log("로컬 docs/motor_profile_test/raw_data/ 로 다운로드 중...")
    subprocess.run(["scp", f"{RPI_HOST}:/tmp/scan_accel_2400.pcd", str(local_pcd)], check=True)
    subprocess.run(["scp", f"{RPI_HOST}:/tmp/scan_accel_2400.json", str(local_json)], check=True)
    
    log(f"★ 다운로드 완료!\n  PCD : {local_pcd} ({local_pcd.stat().st_size} bytes)\n  JSON: {local_json} ({local_json.stat().st_size} bytes)")

if __name__ == "__main__":
    wait_and_collect()
