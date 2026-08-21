# ============================================================================
#  auto_profile_test.ps1 -- 모터 가감속 프로파일 비교 테스트 및 데이터 수집 (PowerShell)
# ----------------------------------------------------------------------------
param (
    [string]$RpiHost = "pi@172.20.26.191",
    [string]$RpiScanDir = "/var/lib/adts/scans"
)

$RepoRoot = Resolve-Path "$PSScriptRoot\.."
$MotorH = Join-Path $RepoRoot "App\motor\motor.h"
$DocsDir = Join-Path $RepoRoot "docs"
$RawDataDir = Join-Path $DocsDir "raw_data"
$ReportFile = Join-Path $DocsDir "motor_profile_test_report.md"

if (-not (Test-Path $RawDataDir)) {
    New-Item -ItemType Directory -Force -Path $RawDataDir | Out-Null
}

function Log-Msg ($msg) {
    Write-Host "[AUTO-TEST] $msg" -ForegroundColor Cyan
}

function Update-MotorH ([int]$startPps) {
    Log-Msg "motor.h 수정 중: MOTOR_START_PPS = ${startPps}u"
    $content = Get-Content $MotorH -Raw
    $newContent = $content -replace '#define\s+MOTOR_START_PPS\s+\d+u', "#define MOTOR_START_PPS           ${startPps}u"
    Set-Content -Path $MotorH -Value $newContent -NoNewline
}

function Build-Firmware {
    Log-Msg "펌웨어 빌드 시작..."
    $buildOut = cmake --build "$RepoRoot\build\Debug" -j8
    if ($LASTEXITCODE -ne 0) {
        Write-Error "빌드 실패!"
        exit 1
    }
    Log-Msg "펌웨어 빌드 성공 (adts.elf)."
}

function Flash-RPi {
    Log-Msg "RPi로 전송 및 OpenOCD 플래시..."
    $flashScript = "$RepoRoot\tools\flash.sh"
    bash $flashScript -H $RpiHost
    if ($LASTEXITCODE -ne 0) {
        Write-Error "플래시 실패!"
        exit 1
    }
    Log-Msg "플래시 및 리셋 완료 (Verified OK)."
}

function Run-RemoteCmd ($cmdStr) {
    ssh -o StrictHostKeyChecking=no $RpiHost $cmdStr
}

function Trigger-Scan {
    Log-Msg "스캔 트리거 실행 (turret_test home & scan)..."
    Run-RemoteCmd "cd ~/final_project/driver && ./turret_test home && sleep 5 && ./turret_test scan"
}

function Wait-ScanDone ($timeoutSec = 600) {
    Log-Msg "스캔 진행 감시 및 완료 대기 중..."
    $startTime = Get-Date
    
    while (((Get-Date) - $startTime).TotalSeconds -lt $timeoutSec) {
        Start-Sleep -Seconds 5
        $stateOut = Run-RemoteCmd "cd ~/final_project/driver && ./turret_test state 2>/dev/null || echo 'UNKNOWN'"
        if ($stateOut -match "state=0" -or $stateOut -match "homed=1") {
            $latest = (Run-RemoteCmd "sudo ls -t $RpiScanDir | head -n 1").Trim()
            if ($latest) {
                Log-Msg "스캔 완료 감지! 최신 파일: $latest"
                Start-Sleep -Seconds 2
                return $latest
            }
        }
        $elapsed = [int]((Get-Date) - $startTime).TotalSeconds
        Log-Msg "스캔 진행 중... (경과 시간: ${elapsed}초)"
    }
    Write-Error "스캔 타임아웃!"
    exit 1
}

function Download-ScanData ($remoteFile, $localName) {
    $localPath = Join-Path $RawDataDir $localName
    Log-Msg "데이터 다운로드: $remoteFile -> $localPath"
    Run-RemoteCmd "sudo cp $RpiScanDir/$remoteFile /tmp/$localName && sudo chmod 666 /tmp/$localName"
    scp "${RpiHost}:/tmp/$localName" $localPath
    Log-Msg "다운로드 완료: $localName"
}

# --- 실행 파이프라인 ---
Log-Msg "=== 모터 가감속 프로파일 A/B 비교 테스트 자동화 시작 ==="

# 1. Phase 1: 가감속 OFF (800 PPS)
Log-Msg "[PHASE 1] Case A : 가감속 OFF (800 PPS 고정)"
Update-MotorH -startPps 800
Build-Firmware
Flash-RPi
Trigger-Scan
$fileA = Wait-ScanDone
Download-ScanData -remoteFile $fileA -localName "scan_no_profile.pcd"

# 2. Phase 2: 가감속 ON (50 PPS)
Log-Msg "[PHASE 2] Case B : 가감속 ON (50 PPS 사다리꼴 램프)"
Update-MotorH -startPps 50
Build-Firmware
Flash-RPi
Trigger-Scan
$fileB = Wait-ScanDone
Download-ScanData -remoteFile $fileB -localName "scan_with_profile.pcd"

Log-Msg "=== 모든 데이터 수집 완료! ==="
Log-Msg "1. docs/raw_data/scan_no_profile.pcd"
Log-Msg "2. docs/raw_data/scan_with_profile.pcd"
