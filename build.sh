#!/usr/bin/env bash
# 编译固件 -> build/obstacle_car.ino.hex
#
# 依赖: arduino-cli + arduino:avr 核心 + AFMotor/Servo 库
# 用法: ./build.sh            # 编译 firmware/obstacle_car
#       ./build.sh path/to/sketch
#
# 项目结束时已删除 tools/ 下的工具链 (约 464MB)。
# 需要重新编译时, 先执行一次 ./setup-toolchain.sh 把工具链装回来。
set -euo pipefail
cd "$(dirname "$0")"

CLI="./tools/arduino-cli.exe"
[ -x "$CLI" ] || CLI="./tools/arduino-cli"

if [ ! -x "$CLI" ]; then
  cat >&2 <<'EOF'
[ERROR] 找不到 arduino-cli。

本项目收尾时删除了 tools/ 下的工具链以释放空间 (约 464MB)。
需要重新编译时, 先执行:

    ./setup-toolchain.sh

它会下载 arduino-cli、安装 arduino:avr 核心和依赖库。
EOF
  exit 1
fi

SKETCH="${1:-firmware/obstacle_car}"
OUT="build"

mkdir -p "$OUT"
"$CLI" --config-file tools/arduino-cli.yaml compile \
  -b arduino:avr:uno \
  --build-path "$OUT/tmp" \
  --output-dir "$OUT" \
  --export-binaries \
  "$SKETCH"

HEX="$OUT/$(basename "$SKETCH").ino.hex"
echo
echo "== 产物 =="
ls -l "$HEX"
echo "== SHA256 =="
sha256sum "$HEX" 2>/dev/null || certutil -hashfile "$HEX" SHA256
