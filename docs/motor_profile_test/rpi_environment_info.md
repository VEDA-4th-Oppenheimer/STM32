# 라즈베리파이(RPi) 연동 및 테스트베드 환경 정보

**기록 일자**: 2026-08-21  
**용도**: STM32 펌웨어 원격 플래싱, LiDAR 스캔 제어, 3D 데이터 수집 및 분석  

---

## 1. 네트워크 및 접속 정보

* **호스트 IP**: `172.20.26.191`
* **SSH 포트**: `22`
* **기본 계정**: `pi`
* **비밀번호**: `1234`
* **SSH 무인 키 인증**: 등록 완료 (`~/.ssh/authorized_keys`에 Windows PC 키 등록됨 $\rightarrow$ `ssh pi@172.20.26.191` 무암호 즉시 접속 가능)

---

## 2. 하드웨어 및 ST-Link 연결

* **ST-Link 디버거**: `STMicroelectronics ST-LINK/V2.1` (`ID 0483:374b`)
* **플래싱 도구**: `OpenOCD (SWD)`
* **플래시 원격 명령어**:
  ```bash
  openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program ~/adts.elf verify reset exit"
  ```
* **모터/센서 인터페이스**: `/dev/turret` (커널 드라이버 `turret_driver.ko`)

---

## 3. 소프트웨어 서비스 및 데이터 경로

### ① 서비스 구성
* **`adts-daemon.service`**: 2축 LiDAR 짐벌 및 STM32 UART 통신 총괄 데몬
* **`adts_web.py`**: Web UI 및 REST API 서버 (포트 `8080`)
  - Web UI 접속: `http://172.20.26.191:8080`

### ② 3D 스캔 데이터 저장 경로
* **경로**: `/var/lib/adts/scans/`
* **산출물 파일 형식**:
  - `calib-YYYYMMDD-HHMMSS_sweep-000001.pcd` (3D 점군 ASCII PCD)
  - `calib-YYYYMMDD-HHMMSS_sweep-000001_pan_tilt_lidar.json` (센서 원시 메타데이터)

---

## 4. 유용한 제어 API 및 CLI 명령어

```bash
# 상태 확인
curl -s http://127.0.0.1:8080/api/state

# 원격 홈(Home) 복귀
curl -X POST http://127.0.0.1:8080/api/cmd/home

# 스캔 시작
curl -X POST http://127.0.0.1:8080/api/cmd/scan

# 안전 정지 (Disarm) 및 재활성화 (Rearm)
curl -X POST http://127.0.0.1:8080/api/cmd/disarm
curl -X POST http://127.0.0.1:8080/api/cmd/rearm
```
