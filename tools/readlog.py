#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
读取小车 EEPROM 里的运行记录。

用法:
    python tools/readlog.py            # 自动找端口
    python tools/readlog.py COM5

注意: 用 dtr=False / rts=False 打开串口, 不会复位 UNO,
所以不会让车重新启动跑起来。发 d 之后固件会停车并打印记录。
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
    ap.add_argument("--clear", action="store_true", help="读完顺手清空记录")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("没找到串口")

    print(f"[readlog] {port} @ {args.baud}")
    ser = serial.Serial(port, args.baud, timeout=0.3)
    # 关掉 DTR/RTS, 避免 UNO 自动复位
    ser.dtr = False
    ser.rts = False
    time.sleep(0.3)
    ser.reset_input_buffer()

    def send(cmd, wait=1.0):
        ser.write((cmd + "\n").encode())
        end = time.time() + wait
        while time.time() < end:
            raw = ser.readline()
            if raw:
                print(raw.decode("utf-8", "replace").rstrip())

    # 先清空接收缓冲, 把正在跑的日志丢掉
    ser.reset_input_buffer()
    send("x", 1.2)      # 停车, 免得它继续跑
    print("---- EEPROM 记录 ----")
    send("d", 3.0)
    if args.clear:
        print("---- 清空 ----")
        send("c", 1.0)

    ser.close()
    print("[readlog] closed (发 g 或拔插 USB 可继续跑)")


if __name__ == "__main__":
    main()
