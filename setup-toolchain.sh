#!/usr/bin/env bash
# =============================================================
#  一次性安装编译工具链
#
#  项目收尾时删除了 tools/ 下的工具链以释放空间 (~464MB)。
#  只有需要重新编译/烧写时才运行这个脚本。
#
#  安装内容:
#    - arduino-cli            (~40MB)
#    - arduino:avr 核心       (~270MB, 含 avr-gcc + avrdude)
#    - Servo 库               (arduino:avr 自带)
#    - AFMotor 库             (已随仓库提供在 vendor/, 不重复下载)
#
#  用法: ./setup-toolchain.sh
# =============================================================
set -euo pipefail
cd "$(dirname "$0")"

CLI_VERSION="${CLI_VERSION:-1.6.0}"
CLI="./tools/arduino-cli.exe"

if [ -x "$CLI" ]; then
  echo "[INFO] arduino-cli 已存在, 跳过下载"
else
  echo "[INFO] 下载 arduino-cli ${CLI_VERSION} ..."
  URL="https://downloads.arduino.cc/arduino-cli/arduino-cli_${CLI_VERSION}_Windows_64bit.zip"
  mkdir -p tools
  curl -fL --retry 3 -o tools/arduino-cli.zip "$URL"
  unzip -o -q tools/arduino-cli.zip -d tools
  rm -f tools/arduino-cli.zip
  chmod +x "$CLI" 2>/dev/null || true
fi

echo "[INFO] 安装 arduino:avr 核心 ..."
"$CLI" --config-file tools/arduino-cli.yaml core update-index
"$CLI" --config-file tools/arduino-cli.yaml core install arduino:avr

# AFMotor 从 vendor/ 装进 arduino-cli 的用户库目录
# (tools/data 与 tools/user 都由 arduino-cli.yaml 指定在本目录内)
LIBDIR="tools/user/libraries/Adafruit-Motor-Shield-library-master"
if [ ! -d "$LIBDIR" ]; then
  echo "[INFO] 安装 AFMotor 库 ..."
  mkdir -p "$(dirname "$LIBDIR")"
  cp -r vendor/Adafruit-Motor-Shield-library-master "$LIBDIR"
else
  echo "[INFO] AFMotor 库已存在, 跳过"
fi

echo
echo "== 完成 =="
"$CLI" --config-file tools/arduino-cli.yaml version
"$CLI" --config-file tools/arduino-cli.yaml core list
echo
echo "现在可以执行 ./build.sh 和 ./flash.bat"
