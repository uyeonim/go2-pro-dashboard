# Go2 대시보드 켜기/끄기 (더블클릭 한 번에 켜고, 한 번 더 누르면 끔)
#  켜기: 대시보드를 창 없이 뒤에서 실행 + Tailscale 공개 주소 켜기 + 브라우저 열기
#  끄기: 대시보드 종료 → 고정 주소(GitHub Pages)는 데모로 열림
# 사용: 대시보드_켜기끄기.bat  (또는 바탕화면 바로가기)
param([switch]$Quiet)   # 알림 창·브라우저 없이 (시험용)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $root ".venv\Scripts\python.exe"
$ts = "C:\Program Files\Tailscale\tailscale.exe"
$logDir = Join-Path $root "logs"
$fixedUrl = "https://uyeonim.github.io/go2-pro-dashboard/"

function Notify($msg, $icon = 64) {
    if ($Quiet) { Write-Output $msg; return }
    # 4초 뒤 자동으로 닫히는 알림 창 (64 = 정보, 48 = 경고)
    (New-Object -ComObject WScript.Shell).Popup($msg, 4, "Go2 대시보드", $icon) | Out-Null
}

function Get-Dashboard {
    Get-CimInstance Win32_Process -Filter "name='python.exe' or name='pythonw.exe'" |
        Where-Object { $_.CommandLine -like '*dashboard.py*' }
}

$running = @(Get-Dashboard)
if ($running.Count -gt 0) {
    # ── 끄기 ──
    $running | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
    Notify "대시보드를 껐어요.`n고정 주소는 이제 데모 화면으로 열려요."
    exit 0
}

# ── 켜기 ──
if (-not (Test-Path $py)) { Notify "가상환경(.venv)을 찾을 수 없어요: $py" 48; exit 1 }
New-Item -ItemType Directory -Force $logDir | Out-Null
$env:PYTHONUTF8 = "1"   # 로그에 이모지가 있어도 깨지지 않게
Start-Process -FilePath $py -ArgumentList "dashboard.py", "--no-browser" -WorkingDirectory $root -WindowStyle Hidden `
    -RedirectStandardOutput (Join-Path $logDir "dashboard_out.log") -RedirectStandardError (Join-Path $logDir "dashboard_err.log")

# Tailscale 공개 주소 켜기 (이미 켜져 있으면 그대로)
$public = $false
if (Test-Path $ts) {
    try {
        $st = & $ts funnel status 2>&1 | Out-String
        if ($st -notmatch "Funnel on") { & $ts funnel --bg 8000 2>&1 | Out-Null }
        $public = ((& $ts funnel status 2>&1 | Out-String) -match "Funnel on")
    } catch { $public = $false }
}

# 서버가 뜰 때까지 최대 15초 기다리기
$ok = $false
for ($i = 0; $i -lt 30; $i++) {
    Start-Sleep -Milliseconds 500
    try { if ((Invoke-WebRequest "http://127.0.0.1:8000/health" -UseBasicParsing -TimeoutSec 2).StatusCode -eq 200) { $ok = $true; break } } catch {}
}
if (-not $ok) { Notify "대시보드를 켜지 못했어요. logs\dashboard_err.log를 확인해 주세요." 48; exit 1 }

if (-not $Quiet) { Start-Process "http://127.0.0.1:8000/" }
if ($public) {
    Notify "대시보드를 켰어요.`n팀원은 고정 주소에서 실시간 화면을 봐요:`n$fixedUrl`n`n끄려면 다시 더블클릭하세요."
} else {
    Notify "대시보드를 켰어요 (이 노트북에서만 보여요).`nTailscale 공개 주소는 켜지 못했어요.`n`n끄려면 다시 더블클릭하세요." 48
}
