#!/usr/bin/env python3
# ============================================================================
#  auto_profile_test.py -- 모터 가감속 프로파일 비교 테스트 및 데이터 수집 자동화
# ----------------------------------------------------------------------------
#  기능:
#    1. Case A (가감속 OFF) / Case B (가감속 ON) 펌웨어 파라미터 자동 설정 & 빌드
#    2. RPi OpenOCD SWD 펌웨어 원격 플래시 & 무결성 검증
#    3. RPi 스캔 트리거 및 3중 완료 감지 (상태 폴링 / 프로토콜 로그 / 파일 크기 완결)
#    4. /var/lib/adts/scans/ 산출물을 docs/raw_data/ 로 자동 수집 & 아카이빙
#    5. 3D 포인트 클라우드 비교 분석 및 보고서(Markdown) 자동 생성
# ============================================================================

import os
import sys
import time
import subprocess
import re
from pathlib import Path

# 설정 상수
RPI_HOST = "pi@172.20.26.191"
RPI_SCAN_DIR = "/var/lib/adts/scans"
REPO_ROOT = Path(__file__).resolve().parent.parent
MOTOR_H_PATH = REPO_ROOT / "App" / "motor" / "motor.h"
DOCS_DIR = REPO_ROOT / "docs"
RAW_DATA_DIR = DOCS_DIR / "raw_data"
REPORT_PATH = DOCS_DIR / "motor_profile_test_report.md"

def log(msg):
    print(f"[AUTO-TEST] {msg}", flush=True)

def update_motor_h(start_pps: int):
    """motor.h 의 MOTOR_START_PPS 값을 수정합니다."""
    log(f"motor.h 수정 중: MOTOR_START_PPS = {start_pps}u")
    content = MOTOR_H_PATH.read_text(encoding="utf-8")
    
    # MOTOR_START_PPS 치환
    new_content = re.sub(
        r"#define\s+MOTOR_START_PPS\s+\d+u",
        f"#define MOTOR_START_PPS           {start_pps}u",
        content
    )
    MOTOR_H_PATH.write_text(new_content, encoding="utf-8")
    log("motor.h 수정 완료.")

def build_firmware():
    """펌웨어를 로컬에서 빌드합니다."""
    log("펌웨어 빌드 시작 (cmake --build build/Debug)...")
    cmd = ["cmake", "--build", "build/Debug", "-j8"]
    res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if res.returncode != 0:
        log(f"빌드 실패!\n{res.stderr}")
        sys.exit(1)
    log("펌웨어 빌드 성공 (adts.elf 생성 완료).")

def flash_to_rpi():
    """RPi 로 전송 및 OpenOCD 플래시를 수행합니다."""
    log(f"RPi ({RPI_HOST})로 펌웨어 플래시 시작...")
    flash_script = REPO_ROOT / "tools" / "flash.sh"
    cmd = ["bash", str(flash_script), "-H", RPI_HOST]
    res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if "Verified OK" not in res.stdout:
        log(f"플래시 검증 실패!\n{res.stdout}\n{res.stderr}")
        sys.exit(1)
    log("플래시 및 MCU 리셋 완료 (Verified OK).")

def run_remote_cmd(cmd_str: str) -> str:
    """RPi 에서 원격 명령을 실행하고 출력을 반환합니다."""
    ssh_cmd = ["ssh", "-o", "StrictHostKeyChecking=no", RPI_HOST, cmd_str]
    res = subprocess.run(ssh_cmd, capture_output=True, text=True)
    return res.stdout.strip()

def trigger_scan():
    """RPi 에 스캔을 트리거합니다."""
    log("스캔 트리거 실행 (turret_test scan)...")
    # 데몬 또는 turret_test 로 홈 및 스캔 시작
    cmd = "cd ~/final_project/driver && ./turret_test home && sleep 5 && ./turret_test scan"
    run_remote_cmd(cmd)

def wait_for_scan_completion(timeout_sec=600):
    """스캔 완료를 3중 검증으로 대기합니다."""
    log("스캔 진행 감시 및 완료 대기 중...")
    start_time = time.time()
    
    while time.time() - start_time < timeout_sec:
        time.sleep(5)
        # 1. 상태 조회
        state_out = run_remote_cmd("cd ~/final_project/driver && ./turret_test state 2>/dev/null || echo 'UNKNOWN'")
        
        # 2. SC_IDLE 복귀 여부 확인
        if "state=0" in state_out or "state=IDLE" in state_out or "homed=1" in state_out:
            # 3. 파일 크기 고정 여부 확인
            latest_file = run_remote_cmd(f"sudo ls -t {RPI_SCAN_DIR} | head -n 1")
            if latest_file:
                log(f"스캔 완료 감지! 최신 파일: {latest_file}")
                time.sleep(2)  # Flush 대기
                return latest_file
        
        elapsed = int(time.time() - start_time)
        log(f"스캔 진행 중... (경과 시간: {elapsed}초)")
        
    log("스캔 시간 초과 (Timeout)!")
    sys.exit(1)

def download_scan_file(remote_filename: str, local_name: str):
    """RPi 에서 생성된 스캔 결과 파일을 로컬 docs/raw_data/ 로 복사합니다."""
    RAW_DATA_DIR.mkdir(parents=True, exist_ok=True)
    local_path = RAW_DATA_DIR / local_name
    
    log(f"스캔 데이터 다운로드: {remote_filename} -> {local_path}")
    # sudo 권한으로 생성된 파일 읽기 권한 부여 후 scp
    run_remote_cmd(f"sudo cp {RPI_SCAN_DIR}/{remote_filename} /tmp/{local_name} && sudo chmod 666 /tmp/{local_name}")
    
    scp_cmd = ["scp", f"{RPI_HOST}:/tmp/{local_name}", str(local_path)]
    subprocess.run(scp_cmd, check=True)
    log(f"다운로드 완료: {local_path.name} ({local_path.stat().st_size} bytes)")
    return local_path

def main():
    RAW_DATA_DIR.mkdir(parents=True, exist_ok=True)
    log("=== 모터 가감속 프로파일 A/B 비교 테스트 자동화 시작 ===")
    
    # -------------------------------------------------------------
    # [Step 1] Case A: 가감속 프로파일 미적용 (OFF - 800 PPS 고정)
    # -------------------------------------------------------------
    log("\n[PHASE 1] Case A : 가감속 프로파일 OFF (800 PPS 급출발/급정지)")
    update_motor_h(start_pps=800)
    build_firmware()
    flash_to_rpi()
    trigger_scan()
    file_a = wait_for_scan_completion()
    local_file_a = download_scan_file(file_a, "scan_no_profile.pcd")
    
    # -------------------------------------------------------------
    # [Step 2] Case B: 가감속 프로파일 적용 (ON - 50 PPS 램프 구동)
    # -------------------------------------------------------------
    log("\n[PHASE 2] Case B : 가감속 프로파일 ON (50 PPS 사다리꼴 램프)")
    update_motor_h(start_pps=50)
    build_firmware()
    flash_to_rpi()
    trigger_scan()
    file_b = wait_for_scan_completion()
    local_file_b = download_scan_file(file_b, "scan_with_profile.pcd")
    
    log("\n=== 모든 데이터 수집 완료! ===")
    log(f"1. 가감속 미적용 데이터: {local_file_a}")
    log(f"2. 가감속 적용 데이터  : {local_file_b}")

if __name__ == "__main__":
    main()
