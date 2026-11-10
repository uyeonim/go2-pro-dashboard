@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONUTF8=1
if "%~2"=="" (
  echo 화면 녹화 영상과 폰 영상, 두 파일을 이 아이콘 위에 한꺼번에 끌어다 놓으세요.
  echo  - 두 영상 모두 '🎬 싱크' 삐-삐-삐 소리가 들어가 있어야 해요
  echo  - 순서는 상관없어요 (세로 영상을 폰 영상으로 봐요)
  pause
  exit /b
)
".venv\Scripts\python.exe" tools\sync_merge.py "%~1" "%~2"
pause
