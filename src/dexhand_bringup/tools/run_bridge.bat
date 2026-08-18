@echo off
REM run_bridge.bat — Windows PC 에서 더블클릭하면 COM15 <-> TCP 5555 브리지가 뜬다.
REM 출력은 화면과 bridge_log.txt 에 동시에 남는다.
REM (PowerShell 을 직접 못 치는 상황을 대비한 우회 경로다. 진행상황 문서 참고)

setlocal
cd /d "%~dp0"

echo === DexHand serial bridge ===
echo COM 포트를 바꾸려면 이 파일의 PORT 값을 고쳐라.
set PORT=COM15
set TCPPORT=5555

python -c "import serial" 2>nul
if errorlevel 1 (
  echo pyserial 설치 중...
  python -m pip install pyserial
)

echo.
echo 브리지 시작: %PORT% ^<-^> TCP %TCPPORT%
echo 창을 닫으면 손은 자동으로 중립 + 출력차단 된다.
echo.

python serial_bridge_win.py --port %PORT% --tcp-port %TCPPORT% > bridge_log.txt 2>&1
type bridge_log.txt
pause
