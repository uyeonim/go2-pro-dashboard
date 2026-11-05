@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 로봇 없이 데모 모드로 대시보드를 시작합니다 (가짜 카메라/센서/LiDAR).
echo (종료하려면 이 창에서 Ctrl+C 또는 창을 닫으세요)
".venv\Scripts\python.exe" dashboard.py --demo
pause
