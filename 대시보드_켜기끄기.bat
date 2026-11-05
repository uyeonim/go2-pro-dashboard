@echo off
rem Double-click to turn the Go2 dashboard on / off (runs hidden in the background)
powershell -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File "%~dp0tools\toggle_dashboard.ps1"
