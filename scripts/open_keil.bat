@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0open_keil.ps1"
exit /b %errorlevel%
