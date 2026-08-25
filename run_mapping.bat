@echo off
cd /d "%~dp0"
set "PYTHON_EXE="
if exist "C:\ProgramData\anaconda3\python.exe" set "PYTHON_EXE=C:\ProgramData\anaconda3\python.exe"
if not defined PYTHON_EXE where python >nul 2>nul && set "PYTHON_EXE=python"
if not defined PYTHON_EXE where py >nul 2>nul && set "PYTHON_EXE=py"
if not defined PYTHON_EXE (
  echo Python was not found. Install 64-bit Python or Anaconda, then run this file again.
  pause
  exit /b 1
)
%PYTHON_EXE% -B main.py
pause
