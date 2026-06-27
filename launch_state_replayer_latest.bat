@echo off
REM ===================================================================
REM  launch_state_replayer_latest.bat
REM
REM  WHAT IT DOES
REM    Finds the newest recording in logs\display_broadcast_recording\*.jsonl.gz
REM    and launches apps.state_replayer with that file.
REM
REM  RUN EXAMPLES
REM    Double-click this file
REM    .\launch_state_replayer_latest.bat
REM    .\launch_state_replayer_latest.bat --dest 127.0.0.1 --port 49200 --loop
REM    .\launch_state_replayer_latest.bat --speed 2.0 --max-gap-s 0.25
REM    .\launch_state_replayer_latest.bat --start-at-s 120
REM
REM  TUNABLES
REM    REPO_DIR - absolute repo path
REM    GAME_PY  - absolute path to conda env python.exe
REM ===================================================================

setlocal

set "REPO_DIR=C:\Users\yck01\GitHub\robot_game_controller"
set "GAME_PY=C:\Users\yck01\miniconda3\envs\game\python.exe"
set "RECORD_DIR=%REPO_DIR%\logs\display_broadcast_recording"
set "DEFAULT_START_AT_S=110"
set "DEFAULT_ARGS=--dest 192.168.0.255 --speed 1.0 --start-at-s %DEFAULT_START_AT_S% --loop"

title Robot Game State Replayer (Latest Recording)

cd /d "%REPO_DIR%"
if errorlevel 1 (
    echo [state_replayer_latest] ERROR: repo folder not found: "%REPO_DIR%"
    pause
    exit /b 1
)

if not exist "%GAME_PY%" (
    echo [state_replayer_latest] ERROR: python not found: "%GAME_PY%"
    pause
    exit /b 1
)

if not exist "%RECORD_DIR%" (
    echo [state_replayer_latest] ERROR: recording folder not found: "%RECORD_DIR%"
    pause
    exit /b 1
)

set "LATEST_FILE="
for /f "delims=" %%F in ('dir /b /a:-d /o-d "%RECORD_DIR%\*.jsonl.gz" 2^>nul') do (
    set "LATEST_FILE=%RECORD_DIR%\%%F"
    goto :found_latest
)

:found_latest
if not defined LATEST_FILE (
    echo [state_replayer_latest] ERROR: no recordings found in "%RECORD_DIR%"
    pause
    exit /b 1
)

set "PYTHONPATH=src"

echo [state_replayer_latest] Using recording:
echo   "%LATEST_FILE%"
echo [state_replayer_latest] Default args:
echo   %DEFAULT_ARGS%
if not "%~1"=="" (
    echo [state_replayer_latest] Extra args - override defaults when repeated:
    echo   %*
)
echo.

"%GAME_PY%" -m apps.state_replayer --file "%LATEST_FILE%" %DEFAULT_ARGS% %*

echo.
echo [state_replayer_latest] Replayer exited with code %errorlevel%.
pause
endlocal
