@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" src\main.py --seed 42 --vehicles 4 --hold
) else if exist "runtime\python.exe" (
  "runtime\python.exe" portable.py --seed 42 --vehicles 4 --hold
) else (
  python src\main.py --seed 42 --vehicles 4 --hold
)
if errorlevel 1 pause
