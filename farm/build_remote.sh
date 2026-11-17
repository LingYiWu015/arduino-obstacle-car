#!/usr/bin/env bash
# =============================================================
#  编译农场 —— 服务器端一键编译 (Linux x86_64 / arm64)
#  - 自动下载 arduino-cli、安装 arduino:avr 内核与依赖库
#  - 编译本项目固件，产出可烧写的 .hex
#  - 产物位置: out/obstacle_car.ino.hex
#
#  用法:
#    ./farm/build_remote.sh                 # 首次会联网准备环境
#    ./farm/build_remote.sh --clean         # 清掉环境后重建
#    ./farm/build_remote.sh firmware/xxx    # 指定 sketch 目录
#
#  服务器无外网时: 把仓库里的 tools/data 打包上传，或设置
#    ARDUINO_DATA_DIR=/path/to/arduino15 复用已装好的环境
# =============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLI_VERSION="${CLI_VERSION:-1.6.0}"
ENV_DIR="${ARDUINO_ENV_DIR:-$ROOT/.farm-env}"
DATA_DIR="${ARDUINO_DATA_DIR:-$ENV_DIR/arduino15}"
OUT_DIR="$ROOT/out"

CLEAN=0
SKETCH=""
for a in "$@"; do
  case "$a" in
    --clean) CLEAN=1 ;;
    *) SKETCH="$a" ;;
  esac
done
SKETCH="${SKETCH:-$ROOT/firmware/obstacle_car}"

if [ "$CLEAN" = "1" ]; then rm -rf "$ENV_DIR"; fi
mkdir -p "$ENV_DIR" "$OUT_DIR"

# ---------- arduino-cli ----------
CLI="$ENV_DIR/arduino-cli"
if [ ! -x "$CLI" ]; then
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64)  ASSET="arduino-cli_${CLI_VERSION}_Linux_64bit.tar.gz" ;;
    Linux-aarch64) ASSET="arduino-cli_${CLI_VERSION}_Linux_ARM64.tar.gz" ;;
    Darwin-*)      ASSET="arduino-cli_${CLI_VERSION}_macOS_64bit.tar.gz" ;;
    MINGW*|MSYS*|CYGWIN*)
      echo "[farm] 这是 Windows。本机请直接用仓库根目录的 build.bat 编译。"
      echo "       本脚本是给 Linux/macOS 服务器用的。"
      exit 1 ;;
    *) echo "unsupported platform: $(uname -s)-$(uname -m)"; exit 1 ;;
  esac
  echo "[farm] downloading $ASSET"
  curl -fsSL -o "$ENV_DIR/cli.tgz" "https://downloads.arduino.cc/arduino-cli/$ASSET"
  tar -xzf "$ENV_DIR/cli.tgz" -C "$ENV_DIR" arduino-cli
  chmod +x "$CLI"
fi

# ---------- 配置：所有状态都放项目内，不污染 $HOME ----------
CFG="$ENV_DIR/arduino-cli.yaml"
cat > "$CFG" <<EOF
directories:
  data: $DATA_DIR
  downloads: $ENV_DIR/downloads
  user: $ENV_DIR/user
library:
  enable_unsafe_install: true
build_cache:
  path: $ENV_DIR/cache
EOF

# ---------- 依赖 ----------
NEED_INSTALL=0
[ -d "$DATA_DIR/packages/arduino/hardware/avr" ] || NEED_INSTALL=1

if [ "$NEED_INSTALL" = "1" ]; then
  echo "[farm] installing arduino:avr core"
  "$CLI" --config-file "$CFG" core update-index
  "$CLI" --config-file "$CFG" core install arduino:avr
fi

# 本地库：优先用仓库自带的 vendor/ 目录（离线可用）
mkdir -p "$ENV_DIR/user/libraries"
if [ -d "$ROOT/vendor/Adafruit-Motor-Shield-library-master" ] && \
   [ ! -d "$ENV_DIR/user/libraries/Adafruit-Motor-Shield-library" ]; then
  cp -r "$ROOT/vendor/Adafruit-Motor-Shield-library-master" \
        "$ENV_DIR/user/libraries/Adafruit-Motor-Shield-library"
fi
"$CLI" --config-file "$CFG" lib list 2>/dev/null | grep -qi '^Servo' || \
  "$CLI" --config-file "$CFG" lib install Servo

# ---------- 编译 ----------
echo "[farm] compiling $(basename "$SKETCH")"
"$CLI" --config-file "$CFG" compile \
  -b arduino:avr:uno \
  --build-path "$ENV_DIR/build-tmp" \
  --output-dir "$OUT_DIR" \
  --export-binaries \
  "$SKETCH"

HEX="$OUT_DIR/$(basename "$SKETCH").ino.hex"
echo
echo "[farm] artifact: $HEX"
ls -l "$HEX"
if command -v sha256sum >/dev/null; then sha256sum "$HEX"; fi
