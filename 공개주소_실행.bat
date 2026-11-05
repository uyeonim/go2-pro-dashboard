@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo [1/2] 대시보드 서버를 새 창에서 시작합니다...
start "Go2 대시보드" ".venv\Scripts\python.exe" dashboard.py --no-browser
timeout /t 5 /nobreak >nul

set "TS=C:\Program Files\Tailscale\tailscale.exe"
if exist "%TS%" goto tailscale

:cloudflare
echo [2/2] Tailscale이 없어서 Cloudflare 임시 주소를 만듭니다.
echo       아래에 나오는 https://....trycloudflare.com 주소를 공유하세요. (실행할 때마다 주소가 바뀝니다)
echo       종료: 이 창에서 Ctrl+C, 대시보드 창도 닫기
"C:\Program Files (x86)\cloudflared\cloudflared.exe" tunnel --no-autoupdate --url http://127.0.0.1:8000
pause
goto :eof

:tailscale
echo [2/2] Tailscale Funnel로 고정 주소를 엽니다.
"%TS%" funnel --bg 8000
if errorlevel 1 (
  echo.
  echo  Funnel을 켜지 못했어요. 위 안내 링크로 Tailscale 관리 화면에서 Funnel을 허용한 뒤 다시 실행하세요.
  pause
  goto :eof
)
echo.
"%TS%" funnel status
echo.
echo  팀 공유용 고정 주소:  https://uyeonim.github.io/go2-pro-dashboard/
echo  (지금처럼 노트북이 켜져 있으면 실제 화면으로, 꺼져 있으면 데모로 이어져요)
echo.
echo  이 창에서 아무 키나 누르면 공개를 끄고 종료합니다. 대시보드 창도 닫으세요.
pause >nul
"%TS%" funnel reset
