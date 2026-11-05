@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo Go2 대시보드를 시작합니다. 잠시 후 브라우저가 열립니다.
echo (종료하려면 이 창에서 Ctrl+C 또는 창을 닫으세요)
".venv\Scripts\python.exe" dashboard.py
pause
