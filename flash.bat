@echo off
REM Flash build\obstacle_car.ino.hex to UNO (default COM5, CH340)
REM Usage: flash.bat          -> uses COM5
REM        flash.bat COM7     -> specify port
setlocal
cd /d "%~dp0"

set PORT=%~1
if "%PORT%"=="" set PORT=COM5

set HEX=build\obstacle_car.ino.hex
if not exist "%HEX%" (
  echo [ERROR] %HEX% not found. Run build.bat first.
  exit /b 1
)

set AVRDUDE=tools\data\packages\arduino\tools\avrdude\8.0.0-arduino1\bin\avrdude.exe
set CONF=tools\data\packages\arduino\tools\avrdude\8.0.0-arduino1\etc\avrdude.conf

if not exist "%AVRDUDE%" (
  echo [ERROR] avrdude not found: %AVRDUDE%
  echo.
  echo Tools were removed to save disk space ^(~464MB^).
  echo Run ./setup-toolchain.sh once to restore the compiler + flasher.
  exit /b 1
)

echo [INFO] port=%PORT%  hex=%HEX%
"%AVRDUDE%" -C "%CONF%" -c arduino -p atmega328p -P %PORT% -b 115200 -D -U flash:w:"%HEX%":i
if errorlevel 1 (
  echo [ERROR] flash failed: check power / port / serial monitor not open
  exit /b 1
)
echo [OK] flash done
endlocal
