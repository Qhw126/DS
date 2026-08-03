@echo off
setlocal enabledelayedexpansion

set ROOT=%~dp0
set OUTDIR=%ROOT%build
if not exist "%OUTDIR%" mkdir "%OUTDIR%"

set CC=arm-none-eabi-gcc
set OBJCOPY=arm-none-eabi-objcopy
set SIZE=arm-none-eabi-size

set CFLAGS=-mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -g -O0 -Wall -Wextra -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
set INC=-I"%ROOT%Start" -I"%ROOT%Library" -I"%ROOT%Hardware" -I"%ROOT%System" -I"%ROOT%User"

set SRCS=
for %%f in ("%ROOT%User\*.c" "%ROOT%System\*.c" "%ROOT%Hardware\*.c" "%ROOT%Library\*.c" "%ROOT%Start\core_cm3.c" "%ROOT%Start\startup_stm32f10x_md_gcc.s" "%ROOT%Start\system_stm32f10x.c") do (
  if /I "%%~xf"==".c" set SRCS=!SRCS! "%%~f"
  if /I "%%~xf"==".s" set SRCS=!SRCS! "%%~f"
  if /I "%%~xf"==".S" set SRCS=!SRCS! "%%~f"
)

%CC% %CFLAGS% %INC% -T"%ROOT%STM32F103C8T6.ld" -Wl,-Map,"%OUTDIR%\Project.map" -Wl,--gc-sections -Wl,--cref -o "%OUTDIR%\Project.elf" %SRCS%
if errorlevel 1 exit /b %errorlevel%

%OBJCOPY% -O ihex "%OUTDIR%\Project.elf" "%OUTDIR%\Project.hex"
%SIZE% "%OUTDIR%\Project.elf"

echo.
echo Build complete: %OUTDIR%\Project.elf
