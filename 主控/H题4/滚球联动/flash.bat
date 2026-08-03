@echo off
setlocal
set ROOT=%~dp0
set ELF=%ROOT%build\Project.elf
if not exist "%ELF%" (
  echo ERROR: %ELF% not found. Please build first.
  exit /b 1
)

where openocd >nul 2>nul
if errorlevel 1 (
  echo ERROR: openocd not found in PATH.
  exit /b 1
)

openocd -f "%ROOT%openocd_stlink.cfg" -c "program %ELF% verify reset exit"
