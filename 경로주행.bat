@echo off
chcp 65001 >nul
cd /d "%~dp0"
set PYTHONUTF8=1
set PY=".venv\Scripts\python.exe"
echo ================================================
echo  Go2 정해진 경로 주행 (로봇 자체 위치 odom 사용)
echo ================================================
echo  준비: 대시보드·ROS·폰 앱은 끄기 / 로봇 세우기(L2+A)
echo        리모컨 손에 들기 (비상정지 L2+B, 멈추기 Ctrl+C)
echo.
echo  1. 앞으로 1m 갔다 돌아오기 (첫 테스트용)
echo  2. 정사각형 1m 한 바퀴
echo  3. 경로 녹화 (리모컨으로 걷기, 끝나면 Ctrl+C)
echo  4. 녹화한 경로 따라 걷기
echo  5. 저장된 경로 목록
echo.
set /p C=번호:
if "%C%"=="1" goto line
if "%C%"=="2" goto square
if "%C%"=="3" goto rec
if "%C%"=="4" goto run
if "%C%"=="5" goto list
goto end

:line
%PY% route_drive.py line --dist 1
goto end

:square
%PY% route_drive.py square --size 1
goto end

:rec
set /p N=경로 이름(영어, 예: lab1):
%PY% route_drive.py record %N%
goto end

:run
%PY% route_drive.py list
set /p N=따라갈 경로 이름:
set L=1
set /p L=몇 바퀴(엔터=1):
%PY% route_drive.py run %N% --loop %L%
goto end

:list
%PY% route_drive.py list

:end
echo.
echo 결과 그림/기록: routes\logs 폴더
pause
