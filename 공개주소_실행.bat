@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo [1/2] 대시보드 서버를 새 창에서 시작합니다...
start "Go2 대시보드" ".venv\Scripts\python.exe" dashboard.py --no-browser
timeout /t 5 /nobreak >nul
echo [2/2] Cloudflare 공개 주소를 만듭니다.
echo       아래에 나오는 https://....trycloudflare.com 주소를 공유하세요. (실행할 때마다 주소가 바뀝니다)
echo       종료: 이 창에서 Ctrl+C, 대시보드 창도 닫기
"C:\Program Files (x86)\cloudflared\cloudflared.exe" tunnel --no-autoupdate --url http://127.0.0.1:8000
pause
