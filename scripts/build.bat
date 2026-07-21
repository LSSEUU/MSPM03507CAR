@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1" -Mode Rebuild
exit /b %errorlevel%
