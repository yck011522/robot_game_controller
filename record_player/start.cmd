@echo off
rem Run from Explorer or any directory: record_player\start.cmd
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0setup.ps1" -Start
if errorlevel 1 pause
