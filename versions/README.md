# 版本说明

`firmware/obstacle_car/obstacle_car.ino` 的历次归档。每个版本都在归档前验证过可编译。

## 最终版

**`v33_turn595.ino`** — 项目结束时的版本，与 `firmware/obstacle_car/obstacle_car.ino` 完全一致。

参数摘要：

```c
baseSpeed = 180, creepSpeed = 70, backSpeed = 180, turnSpeed = 150
trimPct = 23
msPer90Deg = 595, turnLeftBiasMs = 0
stopDistanceCm = 18, clearCm = 20, emergencyCm = 10, slowDistanceCm = 35
sideLookDeg = 90
servoTrimDeg = 3（代码默认；实际以 EEPROM 为准）
```

## 架构分界点：v26

**v26 之前的版本是复杂架构**，包含扇扫、横向平移、斜向探测、多轮瞄准循环。它们在矩形赛道上表现不稳定（表现为"毫无理由地扫来扫去"），已被放弃。

v26 起改为极简决策：测前方 -> 挡路就看左右 -> 转 90 -> 复测。

复杂版最终形态保留在 `v25_stuck_detect.ino`，仅供追溯。

## 完整列表

### 极简架构（v26 ~ v33）

| 版本 | 改动 |
|---|---|
| v33_turn595 | 转弯 645 -> 595ms（按"过头 5~10°"推算，**未实车复测**） |
| v32_banner_fix | 启动横幅改打印真实 PWM（此前一直误显示 L=180 R=180） |
| v31_trim23 | trimPct 20 -> 23 |
| v30_turn645 | 转弯 635 -> 645ms |
| v29_side90 | 侧向探测从 60/75° 两点改为 90° 单点 |
| v28_pct_trim | 配平从固定值改为百分比（解决"越慢越偏"） |
| v27_fast | baseSpeed 100 -> 180，backSpeed 借 turnSpeed 改为独立 |
| v26_simple_rewrite | **架构重写**：删除扇扫/平移/斜向探测 |

### 复杂架构（v1 ~ v25，仅供追溯）

| 版本 | 内容 |
|---|---|
| v25_stuck_detect | 复杂版最终形态：卡死检测 + 598 轮扇扫去重 |
| v24_kick_gate | 修掉"贴墙时全功率助推 200ms"导致冲车 |
| v23_turn_635 | 转弯 651 -> 635ms |
| v22_turn_90_exact | 转弯 615 -> 651ms（发现 615 是按 85° 标的） |
| v21_brake_latency_fix | 测距前必须先停车；近距提前收工 |
| v20_servo_trim_off | 按用户要求注释掉舵机中位修正 |
| v19_wall_vs_corridor | 用斜向探测区分"墙"与"窄通道" |
| v18_servo_recenter | 修掉 `lookAtRel` 测完不回中导致"一直看左边" |
| v17_crab_shift | 引入横向平移（打弯-走-打回） |
| v16_corridor_detect | 走廊模型 / 死胡同判定 |
| v15_faster_sweep | 提速：pulseIn 超时 1000ms -> 25ms |
| v14_move_then_fix | 改成"先移动再修正" |
| v13_side_proxy_fix | 修"进不去了"：侧向门限 20 -> 17 |
| v12_side_clear_15 | 左右各留 15cm |
| v11_escape_and_symmetry | escapeUntil 脱离段 + 左右对称扫描 |
| v10_servo_26_corridor | 舵机中位 26 + 走廊模型 |
| v9_servo_trim_live | 舵机中位串口在线可调 + EEPROM 持久化 |
| v8_corridor_model | 引入走廊抽象（h = 航向偏差，D = 通/死） |
| v7_fast_sweep | 单程扫描，省一半舵机行程 |
| v6_inner_loop_fix | 修最内圈反复修正 |
| v5_sweep_aim_led_eeprom | 扇扫瞄准 + LED 灯码 + EEPROM 记录 |
| v4_spiral_90_verify | 直角螺旋赛道原型 |
| v3_adaptive_angle_26 | 自适应角度 |

## 关于 v33 的 595ms

`msPer90Deg = 595` 是从"645ms 转过头 5~10°"这一次定性反馈反推的：

```
645 × 90 / 97.5 ≈ 595
```

**没有复测验证。** 归档时车已拆下，无法实车确认。

如果后续使用中发现转弯不足（欠转），按 `新值 = 当前值 × 90 / 实测角度` 回调；如果转向仍过头，继续减小。参考历史，真实值大致落在 600~645 之间。

`v32_banner_fix.ino` 保留了 645ms 的完整可运行状态，可直接替换。
