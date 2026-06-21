@echo off
REM ===================================================================
REM  launch_game.bat - double-clickable launcher for the robot game.
REM
REM  WHAT IT DOES
REM    Starts the launcher/supervisor with NO --profile argument, so it
REM    runs whatever profile is currently set as `default_profile` in
REM    config/launcher.yaml (the "current active profile"). To switch
REM    which game launches, edit that one line - no need to touch this
REM    file.
REM
REM  HOW TO USE
REM    Double-click this file (or the Desktop copy). A console window
REM    opens, the subsystems start in tiers, and the window stays open
REM    after exit/crash so you can read any error (press a key to close).
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

title Robot Game Launcher

cd /d "%REPO_DIR%"
if errorlevel 1 (
    echo [launch_game] ERROR: repo folder not found: "%REPO_DIR%"
    pause
    exit /b 1
)

if not exist "%GAME_PY%" (
    echo [launch_game] ERROR: python not found: "%GAME_PY%"
    pause
    exit /b 1
)

REM -- Make 'import core', 'import apps', ... resolve from src/.
set "PYTHONPATH=src"

echo [launch_game] Starting launcher with the active profile from config\launcher.yaml ...
echo.

"%GAME_PY%" -m apps.launcher

echo.
echo [launch_game] Launcher exited with code %errorlevel%.
pause
endlocal
