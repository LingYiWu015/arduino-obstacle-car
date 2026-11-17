#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
小车串口调试终端 (Windows/Linux/macOS 通用, 只需 pyserial)

用法:
    python tools/carctl.py                 # 自动找 CH340/Arduino 端口
    python tools/carctl.py COM5
    python tools/carctl.py COM5 --cmd "x"

交互命令 (对应固件的单字符协议, 见 README §4):
    1 2 3 4     单轮点动测试 8 秒 (M1~M4)
    x           停车并保持
    g           恢复运行
    d           打印 EEPROM 运行记录
    c           清空运行记录 (保留舵机中位)
    +/-         舵机中位 ±1 度, 存 EEPROM
    k           手动扇扫 90/0/180, 用于目视对中
    q           退出

注意:
    - 固件的避障流程是阻塞的, 执行期间串口命令会丢失。
      调参前先发 x 停车。
    - 发送时不用加换行, 固件按单字符解析; 加了也能被忽略掉。
"""
import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("需要 pyserial:  python -m pip install pyserial")


def find_port():
    ports = list(list_ports.comports())
    for p in ports:
        blob = f"{p.device} {p.description} {p.hwid}".lower()
        if any(k in blob for k in ("ch340", "ch341", "arduino", "usb-serial", "usb serial")):
            return p.device
    return ports[0].device if ports else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--cmd", default=None, help="发送一条命令后退出")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("没找到串口，请插上板子或显式指定端口")

    print(f"[carctl] {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.2)
    time.sleep(2.0)  # UNO 打开串口会复位

    def pump(seconds=0.5):
        end = time.time() + seconds
        while time.time() < end:
            raw = ser.readline()
            if raw:
                print(raw.decode("utf-8", "replace").rstrip())

    pump(0.8)

    if args.cmd:
        ser.write(args.cmd.strip().encode())
        pump(1.5)
        return

    print("输入命令, q 退出")
    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line.startswith("!"):
                line = line[1:]     # 跳过前缀, 原样发送
            # 不加换行: 固件是单字符解析, 换行只会被忽略
            ser.write(line.encode())
            pump(0.6)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ser.close()
        print("\n[carctl] closed")


if __name__ == "__main__":
    main()
