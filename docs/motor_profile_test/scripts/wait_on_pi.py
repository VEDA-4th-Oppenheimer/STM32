#!/usr/bin/env python3
import time
import json
import urllib.request
import subprocess
import glob
import os

print("RPi: [Phase 4] 750 PPS 최종 표준 프로파일 실측 스캔 대기 중...", flush=True)
start_time = time.time()

while True:
    try:
        req = urllib.request.urlopen("http://127.0.0.1:8080/api/state", timeout=3)
        data = json.loads(req.read().decode())
        daemon_st = data.get("topics", {}).get("adts/state/daemon", {})
        scanning = daemon_st.get("scanning", False)
        prog = data.get("topics", {}).get("adts/event/progress", {})
        pts = prog.get("points", 0)
        pct = prog.get("percent", 0)
        print(f"진행: {pct:3d}% ({pts:5d}/40200 pts) | 경과: {int(time.time() - start_time)}s", flush=True)
        if not scanning and pts >= 35000:
            print("RPi: [Phase 4] 750 PPS 스캔 완료 감지!", flush=True)
            break
    except Exception as e:
        print(f"에러: {e}", flush=True)
    time.sleep(10)

time.sleep(3)
pcd_files = sorted(glob.glob("/var/lib/adts/scans/*.pcd"), key=os.path.getmtime, reverse=True)
if pcd_files:
    latest_pcd = pcd_files[0]
    latest_json = latest_pcd.replace(".pcd", "_pan_tilt_lidar.json")
    print(f"최신 파일:\n  {latest_pcd}\n  {latest_json}")
    subprocess.run(f"echo 1234 | sudo -S cp {latest_pcd} /tmp/scan_phase4_production_750.pcd", shell=True, check=True)
    subprocess.run(f"echo 1234 | sudo -S cp {latest_json} /tmp/scan_phase4_production_750.json", shell=True, check=True)
    subprocess.run("echo 1234 | sudo -S chmod 666 /tmp/scan_phase4_production_750.*", shell=True, check=True)
    print("RPi: Phase 4 750 PPS 파일 준비 완료 (/tmp/scan_phase4_production_750.*)!")
