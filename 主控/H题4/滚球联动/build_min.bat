@echo off
setlocal enabledelayedexpansion
set ROOT=%~dp0
set OUTDIR=%ROOT%build
if not exist "%OUTDIR%" mkdir "%OUTDIR%"
set CC=D:\ARM-GNU-Toolchain\bin\arm-none-eabi-gcc.exe
set OBJCOPY=D:\ARM-GNU-Toolchain\bin\arm-none-eabi-objcopy.exe
set SIZE=D:\ARM-GNU-Toolchain\bin\arm-none-eabi-size.exe
set CFLAGS=-mcpu=cortex-m3 -mthumb -ffunction-sections -fdata-sections -g -O0 -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
set INC=-I"%ROOT%Start" -I"%ROOT%Library" -I"%ROOT%User"
set SRCS="%ROOT%User\main_min.c" "%ROOT%Start\startup_stm32f10x_md_gcc.s" "%ROOT%Start\system_stm32f10x.c"
%CC% %CFLAGS% %INC% -T"%ROOT%STM32F103C8T6.ld" -Wl,-Map,"%OUTDIR%\Project.map" -Wl,--gc-sections -Wl,--cref -o "%OUTDIR%\Project.elf" %SRCS%
if errorlevel 1 exit /b %errorlevel%
%OBJCOPY% -O ihex "%OUTDIR%\Project.elf" "%OUTDIR%\Project.hex"
%SIZE% "%OUTDIR%\Project.elf"
echo Build complete.
