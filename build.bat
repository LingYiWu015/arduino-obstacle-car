@echo off
REM Build firmware -> build\obstacle_car.ino.hex
REM Requires the toolchain in tools\. If missing, run setup-toolchain.sh once.
setlocal
cd /d "%~dp0"

if not exist "tools\arduino-cli.exe" (
  echo [ERROR] arduino-cli not found.
  echo.
  echo Tools were removed to save disk space ^(~464MB^).
  echo Run ./setup-toolchain.sh once to restore the compiler + flasher.
  exit /b 1
)

tools\arduino-cli.exe --config-file tools\arduino-cli.yaml compile -b arduino:avr:uno --build-path build\tmp --output-dir build --export-binaries firmware\obstacle_car
if errorlevel 1 (
  echo [ERROR] build failed
  exit /b 1
)
echo [OK] build\obstacle_car.ino.hex
endlocal
