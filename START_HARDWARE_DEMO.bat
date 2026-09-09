@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
  ".venv\Scripts\python.exe" src\main.py hardware --mock
) else if exist "runtime\python.exe" (
  "runtime\python.exe" portable.py hardware --mock
) else (
  python src\main.py hardware --mock
)
pause
