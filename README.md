# 超声波避障小车 — 最终版文档

Arduino UNO + Adafruit Motor Shield V1 + HC-SR04 + SG90 的矩形环形赛道小车。
从底部开口进入，绕行一圈后从同一开口驶出。

**当前状态：项目结束。最终固件为 `firmware/obstacle_car/obstacle_car.ino`（对应归档 `versions/v33_turn595.ino`）。**

---

## 1. 硬件

### 1.1 接线

| 模块 | 引脚 | 说明 |
|---|---|---|
| HC-SR04 Trig | A0 | 输出触发脉冲 |
| HC-SR04 Echo | A1 | 输入回波脉宽 |
| SG90 信号 | D10 | 由 Servo 库驱动 |
| 板载 LED | D13 | 状态灯码，见 §5 |

扩展板占用 D4/D5/D6/D7/D8/D11/D12，舵机口 D9/D10，其余不要接。

### 1.2 电机

| 通道 | 位置 | 备注 |
|---|---|---|
| M1 | 左前 | |
| M2 | 左后 | |
| M3 | 右后 | 动力偏弱，见 §7 |
| M4 | 右前 | |

### 1.3 供电

舵机**不要**吃 UNO 的 5V，独立供电、GND 共地。堵转瞬间会把 UNO 拉复位。

### 1.4 L293D 注意事项

- 自身压降约 2V，实际到电机电压低于电池电压
- `run(BRAKE)` 在该库中是**空实现**（`AFMotor.cpp` 的 `run()` 只处理 FORWARD/BACKWARD/RELEASE）
- 刹车只能用 `RELEASE`：两输入全低 = 电机短接 = 动态制动

---

## 2. 编译与烧写

### 2.1 首次使用：安装工具链

项目收尾时删除了 `tools/` 下的工具链（约 464MB）以释放磁盘空间。
**只有需要重新编译/烧写时才需要装回来。**

```bash
./setup-toolchain.sh
```

会下载 arduino-cli 1.6.0、安装 `arduino:avr` 核心（含 avr-gcc 与 avrdude）。
AFMotor 库从仓库自带的 `vendor/` 复制，不重复下载。

### 2.2 编译与烧写

```bat
build.bat                 :: 编译 -> build\obstacle_car.ino.hex
flash.bat                 :: 烧写到 COM5
flash.bat COM7            :: 指定端口
```

Linux / 服务器：

```bash
./build.sh                # 编译
./farm/build_remote.sh    # 服务器一键编译，产物 out/
```

成功标志：

```
Sketch uses 8650 bytes (26%) of program storage space.
Global variables use 753 bytes (36%) of dynamic memory.
```

烧写成功：

```
8650 bytes of flash verified
[OK] flash done
```

**不要用** `build/*.with_bootloader.hex`，会覆盖 bootloader。

### 2.3 已编译好的固件

`build/obstacle_car.ino.hex` 已随仓库提供，**不需要工具链**也能烧写 —— 但 `flash.bat`
依赖 avrdude，仍需先跑一次 `setup-toolchain.sh`。也可以用 Arduino IDE 直接打开 .hex 烧写。

---

## 3. 运行行为

### 3.1 决策流程

```
loop():
  measureFront()                        前方测距（3 次取中位数）
    ├─ 无回波          -> 爬行试探（creepSpeed）
    ├─ <= 10cm         -> HardStop + avoidObstacle()
    ├─ escapeUntil 中  -> 爬行脱离（刚转完弯，不在墙角反复触发）
    └─ < 18cm          -> HardStop + avoidObstacle()
       否则             -> 前进（<35cm 用 creepSpeed，>=35cm 用 baseSpeed）

avoidObstacle():
  左右各看一次（舵机摆到正侧方）-> 选更宽的一边
  -> 倒退 75ms 让出转弯空间
  -> 转 90 度 -> 测前方
       通了 -> 结束
       还窄 -> 再转 90 度（最多 3 次，够掉头）
  -> 记录 EEPROM -> 设置 escapeUntil
```

没有扇扫、没有斜向探测、没有内部多轮循环。一次避障最多 5 次测距。

### 3.2 关键参数

```c
const int baseSpeed     = 180;   // 直行
const int creepSpeed    = 70;    // 近距爬行
const int backSpeed     = 180;   // 后退
const int turnSpeed     = 150;   // 转弯

const int trimPct       = 23;    // 右侧比左侧快 23%（补偿右轮偏弱）

const int msPer90Deg    = 595;   // 转 90 度所需通电时间
const int turnLeftBiasMs = 0;    // 左转额外扣时（当前未启用）

const int stopDistanceCm = 18;   // 障碍判定
const int clearCm        = 20;   // 认为是通路
const int emergencyCm    = 10;   // 无条件硬停
const int slowDistanceCm = 35;   // 降速带

const int sideLookDeg   = 90;    // 左右探测角
const int backupMs      = 75;    // 转向前倒退
const int escapeMs      = 400;   // 转完后爬行脱离时长
const int turnSettleMs  = 120;   // 转完等车身停稳
const int maxTurnSteps  = 3;     // 连续盲转上限
```

---

## 4. 串口协议

波特率 **9600**。

| 命令 | 作用 |
|---|---|
| `1` `2` `3` `4` | 单轮点动测试 8 秒（M1~M4），结束后自动停车 |
| `x` | 停车并保持（holdStop） |
| `g` | 恢复运行 |
| `d` | 打印 EEPROM 运行记录（会先停车） |
| `c` | 清空运行记录（保留舵机中位） |
| `+` / `-` | 舵机中位 ±1 度，立即存 EEPROM |
| `k` | 手动扇扫（90 → 0 → 180 → 90），用于目视对中 |

### 4.1 重要注意事项

**`checkDirection()` / `avoidObstacle()` 执行期间串口命令会丢失。** 避障是阻塞流程，期间不读串口。调参前先发 `x` 停车。

**EEPROM 的舵机中位会覆盖代码里的默认值。** 开机流程：

```
loadTrim();        // EEPROM 有有效值(magic 0x5A)就覆盖 servoTrimDeg
```

如果改了代码里的 `servoTrimDeg` 但没生效，先发 `c` 清 EEPROM，或直接用 `+`/`-` 调。

### 4.2 无串口时的状态灯

| 灯码 | 含义 |
|---|---|
| 1 短闪 | 往左转 |
| 2 短闪 | 往右转 |
| 快速 5 闪 | 转完仍然很窄 |
| 长亮一次 | 转完前方通畅 |

---

## 5. 舵机中位校准

`servoTrimDeg`：**正数 = 探头往左**。

写入逻辑：

```c
void ServoWrite(int deg) {
  int v = deg + servoTrimDeg;
  if (v < 0) v = 0;
  if (v > 180) v = 180;
  servo.write(v);
}
```

### 5.1 校准方法

1. 发 `k`，看舵机停在 0 / 90 / 180 时探头朝向
2. 或发 `x` 停车后用 `+` / `-` 微调
3. 每次 1 度，保存后立即生效

### 5.2 当前值

```c
int servoTrimDeg = 3;    // 代码默认
```

**实际生效值以开机横幅为准**，可能是 EEPROM 里的值：

```
trim loaded from EEPROM: 7
```

### 5.3 侧向探测的边界

`lookSide()` 会按中位修正限制可用角度：

```c
int lim = 90 - abs(servoTrimDeg);
```

`servoTrimDeg` 越大，侧向探测范围越窄，左右视野越不对称。超过 20 左右就该考虑改机械安装，而不是继续用代码补。

### 5.4 SG90 角度超程

标称 90±90°，实测物理行程约 180° 偏大（常见 190~200°）。这是正常现象，不是故障。

---

## 6. EEPROM 布局

UNO 有 1KB。

| 地址 | 内容 |
|---|---|
| 0~1 | 记录计数器（uint16） |
| 8~，每条 8 字节 | 运行记录，最多 127 条 |
| 1022 | 舵机中位 magic（0x5A） |
| 1023 | 舵机中位值（int8） |

记录格式：

```
A5 | 左 | 右 | 前方 | 方向(1左/2右) | 转了几次 | 保留 | 保留
```

读取：

```bash
python tools/readlog.py            # 自动找端口
python tools/readlog.py COM5
python tools/readlog.py COM5 --clear
```

`readlog.py` 用 `dtr=False; rts=False` 打开串口，**不会复位 UNO**，所以读日志不会让车重新跑起来。

---

## 7. 已知问题与设计取舍

### 7.1 右轮偏弱

`trimPct = 23` 是在 PWM 上补偿右轮偏弱。这是**用软件掩盖硬件问题**。

副作用：直行时左右是 180:221，而转弯 `TurnSpeed()` 不带配平（四轮同速 150）。两种模式下左右轮相对关系不一致。

**如需彻底解决**，应检查右轮：减速箱阻力、接线接触、电机本身。修好硬件后把 `trimPct` 降到 0。

### 7.2 无航向闭环

没有编码器/陀螺仪，`msPer90Deg` 只能靠时间标定。标定值会随**电池电压、地面摩擦、轮胎状态**漂移。

标定历史（可见漂移幅度）：

| 值 | 实测 |
|---|---|
| 615 | 85°（欠） |
| 635 | 略欠 |
| 645 | 过 5~10° |
| 651 | 92~94°（过） |
| **595** | 当前值，按 645 过头 7.5° 反推，**未实车复测** |

635 → 645 只差 10ms（约 1.5°），结果却从"欠"跳到"过 5~10°"，说明单靠时间标定精度有限。

**重新标定公式：**

```
新值 = 当前值 × 90 / 实测角度
```

### 7.3 左转比右转猛

`turnLeftBiasMs` 是为这个问题准备的，当前设为 **0（未启用）**。

原因推测：右轮偏弱，右转时弱轮在**倒退**，提供的反作用力矩更小，所以右转偏软、左转偏猛。

如果要启用，从 15~20 开始试。但需先确认现象是"左转这个动作过头"而不是"总是选左边"—— 后者是 `lookSide()` 判断问题，调这个参数是治错的。

### 7.4 放弃的复杂逻辑

以下逻辑曾尝试过，因稳定性问题在 v26 重写时移除：

| 逻辑 | 移除原因 |
|---|---|
| 扇扫统 11 点求走廊轴线 | 矩形赛道上 argmax 常在两格间振荡 |
| 横向平移修正（打弯-走-打回） | 无编码器，平移量靠时间估算不可靠 |
| 斜向探测判断窄通道 | 墙角里总是得出"是墙"，纯属白扫 |
| 转向后原地航向修正 | 只改朝向不改横向位置，累积误差变成固定斜角 |

保留在 `versions/v25_stuck_detect.ino` 及更早版本中，供追溯。

### 7.5 右侧测距异常

历史日志中多次出现右侧读数远小于左侧（如 `L=74 R=0`、`L=36 R=9`）。原因是单条射线打在挡板接缝上。当前用"每方向单次探测 + 转完复测"兜底，不再做多角度取最大值。

---

## 8. 上车检查顺序

1. **架空车轮**，串口看 `Distance:` 是否稳定，不应出现大量 0
2. **手挡超声波** 10cm，看是否打印 `EMERGENCY` 并开始转向
3. **发 `1`~`4`** 逐个确认电机对应位置正确、都能转起来
4. **发 `k`** 目视舵机中位是否正对前方，偏了用 `+`/`-` 调
5. **放地上跑直路**，看是否偏向；偏右调大 `trimPct`，偏左调小
6. **跑完整圈**，观察转弯角度是否需要重标 `msPer90Deg`

---

## 9. 版本归档

`versions/` 保留了全部 31 个迭代版本，每个都验证过可编译。

| 版本 | 内容 |
|---|---|
| **v33_turn595** | **最终版**（= `firmware/obstacle_car/obstacle_car.ino`） |
| v32_banner_fix | 启动横幅打印真实 PWM |
| v31_trim23 | 配平改为百分比 23% |
| v30_turn645 | 转弯 645ms |
| v29_side90 | 侧向探测改为 90°单次 |
| v28_pct_trim | 配平改为百分比制 |
| v27_fast | 提速到 180 |
| v26_simple_rewrite | **架构重写点**：砍掉扇扫/平移/斜向探测 |
| v25_stuck_detect | 最后一代复杂版 |

v26 之前的版本为复杂架构，仅供追溯，不建议使用。

---

## 10. 运行时需求（依赖）

| 项 | 要求 |
|---|---|
| 开发板 | Arduino UNO (ATmega328P) |
| FQBN | `arduino:avr:uno` |
| 核心 | arduino:avr |
| 库 | `Servo`（内置）、`EEPROM`（内置）、`AFMotor`（`vendor/Adafruit-Motor-Shield-library-master`） |
| 编译工具 | arduino-cli 1.6.0（**需 `./setup-toolchain.sh` 安装**） |
| 烧写工具 | avrdude 8.0.0-arduino1（同上） |
| 串口脚本 | Python 3 + pyserial |
| 烧写端口 | 默认 COM5（CH340），可用 `flash.bat COMx` 指定 |

固件资源占用：

```
Flash  8650 / 32256 bytes  (26%)
RAM     753 /  2048 bytes  (36%)
```

### 10.1 磁盘占用说明

项目收尾时做了空间清理，仓库本体只有约 **1.4MB**：

| 部分 | 大小 | 说明 |
|---|---|---|
| `firmware/` | 20K | 固件源码 |
| `versions/` | 820K | 31 个历史版本归档 |
| `vendor/` | 69K | AFMotor 库 |
| `build/` | 56K | 已编译的 .hex |
| `tools/` | 416K | 串口脚本 + arduino-cli 配置 |

被删除的部分（用 `setup-toolchain.sh` 可完整恢复）：

| 已删除 | 原大小 |
|---|---|
| `tools/data/`（核心 + avrdude + gcc） | 331MB |
| `tools/downloads/` | 72MB |
| `tools/arduino-cli.exe` | 40MB |
| `tools/arduino-cli.zip` | 20MB |
| `build/tmp/`（编译中间产物） | 1.3MB |

**没有在其他位置留下 Arduino 相关文件** —— 工具链原本就全部位于项目目录内，未污染系统环境或用户目录（`AppData`、`Documents/Arduino`、`~/.arduino15` 均无残留）。
