#!/usr/bin/env python3
import time
import urllib.request
import json
import subprocess
import glob
import os
import math

def wait_scan():
    print("RPi: Case A 스캔 완료 대기 중...", flush=True)
    while True:
        try:
            req = urllib.request.urlopen("http://127.0.0.1:8080/api/state", timeout=3)
            data = json.loads(req.read().decode())
            daemon_st = data.get("topics", {}).get("adts/state/daemon", {})
            scanning = daemon_st.get("scanning", False)
            prog = data.get("topics", {}).get("adts/event/progress", {})
            pts = prog.get("points", 0)
            pct = prog.get("percent", 0)
            print(f"  -> 진행률: {pct}% ({pts}/40200 points), scanning={scanning}", flush=True)
            if not scanning and pts >= 30000:
                print("RPi: Case A 스캔 완료 감지!", flush=True)
                break
        except Exception as e:
            print(f"조회 에러: {e}", flush=True)
        time.sleep(4)

    time.sleep(2)
    pcd_files = sorted(glob.glob("/var/lib/adts/scans/*.pcd"), key=os.path.getmtime, reverse=True)
    json_files = sorted(glob.glob("/var/lib/adts/scans/*.json"), key=os.path.getmtime, reverse=True)
    if pcd_files:
        latest_pcd = pcd_files[0]
        latest_json = json_files[0]
        print(f"RPi: 최신 파일 복사: {latest_pcd}")
        subprocess.run(f"echo 1234 | sudo -S cp {latest_pcd} /tmp/scan_no_profile.pcd", shell=True, check=True)
        subprocess.run(f"echo 1234 | sudo -S cp {latest_json} /tmp/scan_no_profile.json", shell=True, check=True)
        subprocess.run("echo 1234 | sudo -S chmod 666 /tmp/scan_no_profile.*", shell=True, check=True)
        print("RPi: 파일 준비 완료!")

if __name__ == "__main__":
    wait_scan()
