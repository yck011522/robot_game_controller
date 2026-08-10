@echo off
REM ===================================================================
REM  launch_game_longer_duration.bat - launcher for the 10-minute game.
REM
REM  WHAT IT DOES
REM    Starts the launcher/supervisor with `--profile two_teams_longer_duration`
REM    so it loads `config/profiles/two_teams_longer_duration.yaml`, which has a
REM    600-second (10-minute) gameplay duration instead of the standard 300-second duration.
REM
REM  HOW TO USE
REM    Double-click this file (or a Desktop shortcut). A console window
REM    opens, the subsystems start in tiers, and the window stays open
REM    after exit/crash so you can read any error (press a key to close).
REM
REM  TYPICAL RUN COMMANDS
REM    From PowerShell or Command Prompt:
REM      .\launch_game_longer_duration.bat
REM    Or directly via Python launcher:
REM      python -m apps.launcher --profile config\profiles\two_teams_longer_duration.yaml
REM
REM  TUNABLES (edit the two SET lines below if your install moves)
REM    REPO_DIR  - absolute path to the repo root.
REM    GAME_PY   - absolute path to the conda 'game' env python.exe.
REM ===================================================================

setlocal

REM -- Repo root (where config/, src/ live). Change if you move the repo.
set "REPO_DIR=C:\Users\yck01\GitHub\robot_game_controller"

REM -- Python interpreter of the conda 'game' environment.
set "GAME_PY=C:\Users\yck01\miniconda3\envs\game\python.exe"

title Robot Game Launcher (600s Longer Duration)

cd /d "%REPO_DIR%"
if errorlevel 1 (
    echo [launch_game_longer_duration] ERROR: repo folder not found: "%REPO_DIR%"
    pause
    exit /b 1
)

if not exist "%GAME_PY%" (
    echo [launch_game_longer_duration] ERROR: python not found: "%GAME_PY%"
    pause
    exit /b 1
)

REM -- Make 'import core', 'import apps', ... resolve from src/.
set "PYTHONPATH=src"

echo [launch_game_longer_duration] Starting launcher with profile config\profiles\two_teams_longer_duration.yaml ...
echo.

"%GAME_PY%" -m apps.launcher --profile config\profiles\two_teams_longer_duration.yaml

echo.
echo [launch_game_longer_duration] Launcher exited with code %errorlevel%.
pause
endlocal
