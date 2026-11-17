// =====================================================================
// 简化版避障小车
//
// 上一版为什么会"毫无理由地扫来扫去", 三个原因叠在一起:
//
//   1) 每轮瞄准都要扇扫 11 个点 (±75 度), 一次避障最多 4 轮 = 44 次摆动
//   2) 扇扫算完, 又调用 lookSide 重复扫 45/60/75 这 3 个点 (本来就扫过)
//   3) 前方 < 18cm 时还要额外做两次斜向探测 (lookAtRel ±30) 判断"窄通道"
//      而这两次探测几乎总是得出"是墙"的结论 —— 纯粹白扫
//
// 结果: 车在一个墙角里可以摆动几十次, 看起来就是原地乱扫。
//
// 这一版把决策压到最少:
//   - 平时只测正前方
//   - 被挡住 -> 左右各看一次 (2 次摆动) -> 选宽的一边 -> 转 90 度
//   - 转完还挡着 -> 再转 90 度 (最多 3 次, 够掉头)
//   - 不再有扇扫, 不再有斜向探测, 不再有内部的 4 轮循环
//
// 代价: 放弃"横向平移修正"。但没有编码器/陀螺仪的情况下,
// 平移量靠时间估算本身就不可靠, 换来的是行为可预测。先跑通再谈精度。
// =====================================================================

#include <Servo.h>
#include <AFMotor.h>
#include <EEPROM.h>

#define Trigpin A0
#define Echopin A1
#define LED_PIN 13

Servo servo;
AF_DCMotor motor1(1);   // 实测: 左前
AF_DCMotor motor2(2);   // 实测: 左后
AF_DCMotor motor3(3);   // 实测: 右后
AF_DCMotor motor4(4);   // 实测: 右前

// ---------------- 参数 ----------------
// 速度上限受 trimRight 限制: 左侧速度 = baseSpeed, 右侧 = baseSpeed + trimRight,
// 右侧超过 255 会被夹住, 左右比例就崩了 (车往一边偏)。
// 配平比例是 trimRight/baseSpeed = 38/100 = 0.38, 要放大速度必须同比例放大配平:
//   baseSpeed * 1.38 <= 255  =>  baseSpeed <= 184
// 取 180 (右侧 248), 给 PWM 留一点余量, 避免长期贴着饱和点。
const int baseSpeed = 180;      // 直行 (原 100)
const int creepSpeed = 70;      // 近距爬行 (安全速度, 不提)

// 倒车单独给速度。原来 Backward() 借用 turnSpeed(150), 那是转弯用的,
// 倒车和转弯的负载完全不同, 分开标更好调。
const int backSpeed = 180;      // 后退 (原 借用 turnSpeed=150)

// 直行配平: 右轮偏弱, 右侧按比例提速。
//
// 这里必须是"比例", 不能是固定值。
// 固定值的老写法 (trimRight = 38) 在 baseSpeed=100 时是 100/138 (1.38),
// 但爬行 creepSpeed=70 时变成 70/108 (1.54), 速度越低右边越强。
// 把 baseSpeed 提到 180 后放大成 trimRight=68, 爬行时就成了 70/138 (1.97) ——
// 右边直接是左边的两倍, 车在贴墙慢行时会明显往左甩。
//
// 改成百分比后, 任何速度下左右比例都一致:
//   180 -> 216 (1.20)
//    70 ->  84 (1.20)
// 需要调就改这一个数, 不用管当前是什么速度档。
//
// 标定方法: 直线跑 2 米, 看车头偏向。
//   往右偏 -> 这个值调大; 往左偏 -> 调小。每次改 3~5。
const int trimPct = 23;      // 右侧比左侧快 23% (原 20, 实测仍偏左一点)

// 保留绝对值配平口子 (置 0 表示不用), 方便临时手动偏置
const int trimLeft  = 0;
const int trimRight = 0;

// 转弯标定: msPer90Deg = "转出 90 度所需的通电时间"。
// 公式: 新值 = 当前值 * 90 / 实测角度
//
// 历史:
//   615 -> 约 85 度 (欠)
//   651 -> 约 92~94 度 (过)
//   635 -> 略欠
//   645 -> 实测转过头 5~10 度, 即约 95~100 度 (取中值 97.5)
//          => 645 * 90 / 97.5 = 595
//
// 注意: 645 只比 635 多 10ms (约 1.5 度), 却从"略欠"跳到"过 5~10 度",
// 说明这个标定本身在漂 —— 电池电压、地面、轮胎状态都会影响。
// 所以不要指望一次调准, 每次只动 10~20ms, 试完立刻反馈。
const int turnSpeed  = 150;
const int msPer90Deg = 595;

// 左转单独扣一点时间。
// 实测左转总是比右转多转, 原因不只是标定误差:
//   左转 = 左侧前进 + 右侧后退
//   右转 = 左侧后退 + 右侧前进
// 而右轮本来就偏弱 (所以才要 trimPct), 弱轮"倒退"时提供的反作用力矩更小,
// 于是右转偏软、左转偏猛 —— 这是硬件不对称, 不能靠一个全局 msPer90Deg 修。
// 每次左转少给这么多毫秒。0 = 不修正。
// 调法: 左转还是过头就加 5~10, 变成欠转就减。
const int turnLeftBiasMs = 0;

// 距离门限
const int stopDistanceCm = 18;   // 小于这个值就是障碍
const int clearCm        = 20;   // 大于等于这个值认为通行
const int emergencyCm    = 10;   // 极近距离, 立刻硬停
const int slowDistanceCm = 35;   // 小于这个值降速

// 避障动作
const int sideLookDeg   = 90;    // 左右各看 90 度 (正侧方, 不用斜角)
const int backupMs      = 75;    // 转向前倒退一点
const int escapeMs      = 400;   // 转完后的脱离时间
const int turnSettleMs  = 120;   // 转完等车身停止旋转
const int maxTurnSteps  = 3;     // 连续盲转上限 (3 x 90 = 掉头)

// 舵机中位修正 (正数 = 往左修), 存在 EEPROM
int servoTrimDeg = 3;

// ---------------- 状态 ----------------
bool isMoving = false;
int  testNum  = 0;
unsigned long testUntil = 0;
bool holdStop = false;
unsigned long escapeUntil = 0;
unsigned long lastTrigUs = 0;
int curServoDeg = 90;

// ---------------- 基础动作 ----------------
// 直行: 右轮按比例提速。
// 比例而非固定值, 这样 180 直行和 70 爬行时左右比例一致,
// 不会出现"越慢越偏"的现象。
void Speed(int s)
{
  int l = s + trimLeft;                    // M1 左前, M2 左后
  int r = s + (int)((long)s * trimPct / 100) + trimRight;   // M3 右后, M4 右前

  if (l < 0) l = 0; if (l > 255) l = 255;
  if (r < 0) r = 0; if (r > 255) r = 255;

  motor1.setSpeed(l);
  motor2.setSpeed(l);
  motor3.setSpeed(r);
  motor4.setSpeed(r);
}

void TurnSpeed(int s)
{
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  motor1.setSpeed(s);
  motor2.setSpeed(s);
  motor3.setSpeed(s);
  motor4.setSpeed(s);
}

// L293D 两输入全低 = 电机短接 = 动态制动 (库里的 BRAKE 是空实现)
void Stop()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  isMoving = false;
}

void Forward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void Backward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  TurnSpeed(backSpeed);   // 倒车用独立速度, 不再借用转弯速度
}

void TurnLeft()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  TurnSpeed(turnSpeed);
}

void TurnRight()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  TurnSpeed(turnSpeed);
}

void Brake()
{
  Stop();
  delay(100);
}

// 障碍急停: 速度归零 -> RELEASE -> 等惯性滑完
void HardStop()
{
  Speed(0);
  Stop();
  delay(200);
}

// 所有前进都走这里。
//
// 这里原来有一段"起步助推": Speed(255) + delay(200)。
// 在 baseSpeed=180 下它必须删掉, 原因是:
//   右侧速度 = 255 + trimRight(68) = 323 -> 被夹到 255
//   左侧速度 = 255 + 0 = 255
//   于是助推的那 200ms 里左右是 1:1, 而平时是 180:248 ——
//   助推反而变成偏航源, 车会在每次起步时往左甩一下。
//
// 而且 180 的 PWM 起步本身就不需要冲刺, 助推原本是为了克服
// 低速(70)下的静摩擦, 现在只在爬行分支里需要。
void StartMove(int speed, int front)
{
  (void)front;      // 保留参数, 以后若要恢复助推会用到
  Forward();

  // 低速起步助推: 只在爬行时用。
  // 助推速度同样走 Speed(), 所以左右比例和平时一致, 不会甩。
  if (!isMoving && speed <= creepSpeed) {
    int kickTop = 120;
    if (kickTop <= speed) kickTop = speed + 20;
    Speed(kickTop);
    delay(150);
  }

  isMoving = true;
  Speed(speed);
}

void turnByDeg(bool left, int deg)
{
  int ms = (int)((long)msPer90Deg * deg / 90);

  // 左转单独扣时间 (硬件左右不对称, 见 turnLeftBiasMs 的说明)
  if (left) ms -= turnLeftBiasMs;
  if (ms < 0) ms = 0;

  if (left) TurnLeft();
  else      TurnRight();
  delay(ms);
  Brake();
  delay(turnSettleMs);   // 等车身真的停下, 否则紧接着测的是"还在转"的画面
}

// ---------------- 舵机中位 ----------------
#define TRIM_ADDR 1022

void loadTrim()
{
  uint8_t magic = EEPROM.read(TRIM_ADDR);
  int8_t  v     = (int8_t)EEPROM.read(TRIM_ADDR + 1);
  if (magic == 0x5A && v >= -90 && v <= 90) {
    servoTrimDeg = v;
    Serial.print("trim loaded from EEPROM: ");
    Serial.println(servoTrimDeg);
  }
}

void saveTrim()
{
  EEPROM.update(TRIM_ADDR, 0x5A);
  EEPROM.update(TRIM_ADDR + 1, (uint8_t)(int8_t)servoTrimDeg);
  Serial.print("trim saved: ");
  Serial.println(servoTrimDeg);
}

// ---------------- 板载反馈 ----------------
void blinkN(int n, int onMs, int offMs)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(onMs);
    digitalWrite(LED_PIN, LOW);  delay(offMs);
  }
}

// ---------------- EEPROM 记录 ----------------
// 格式 (8 字节): A5 | 左 | 右 | 前方 | 方向 | 修正次数 | 保留 | 保留
#define REC_ADDR 8
#define REC_SIZE 8
#define REC_MAX  ((1024 - REC_ADDR) / REC_SIZE)

void logEvent(int l, int r, int f, bool wentLeft, int steps, int al, int ar)
{
  uint16_t n;
  EEPROM.get(0, n);
  if (n == 0xFFFF) n = 0;

  int a = REC_ADDR + (int)(n % REC_MAX) * REC_SIZE;
  EEPROM.update(a + 0, 0xA5);
  EEPROM.update(a + 1, (uint8_t)constrain(l, 0, 254));
  EEPROM.update(a + 2, (uint8_t)constrain(r, 0, 254));
  EEPROM.update(a + 3, (uint8_t)constrain(f, 0, 254));
  EEPROM.update(a + 4, wentLeft ? 1 : 2);
  EEPROM.update(a + 5, (uint8_t)steps);
  EEPROM.update(a + 6, (uint8_t)constrain(al, 0, 254));
  EEPROM.update(a + 7, (uint8_t)constrain(ar, 0, 254));

  n++;
  EEPROM.put(0, n);
}

void dumpLog()
{
  uint16_t n;
  EEPROM.get(0, n);
  if (n == 0xFFFF) n = 0;

  int cnt = (n < REC_MAX) ? (int)n : REC_MAX;
  Serial.print("LOG count="); Serial.print(n);
  Serial.print(" slots="); Serial.println(cnt);
  Serial.println("L R front dir steps a b");

  for (int i = 0; i < cnt; i++) {
    int a = REC_ADDR + i * REC_SIZE;
    if (EEPROM.read(a) != 0xA5) continue;
    for (int k = 1; k < REC_SIZE; k++) {
      Serial.print((int)EEPROM.read(a + k));
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println("LOG end");
}

// ---------------- 测距 ----------------
// 超时 25000us (约 4 米)。pulseIn 默认超时是 1 秒, 读不到回波会死等,
// 这是以前"检查太慢"的主因。
#define ECHO_TIMEOUT_US 25000

long HC_SR04()
{
  // HC-SR04 两次触发间隔需 >= 60ms, 这里"补足到" 60ms 而不是固定 delay
  unsigned long since = micros() - lastTrigUs;
  if (since < 60000UL) {
    delayMicroseconds(60000UL - since);
  }

  digitalWrite(Trigpin, LOW);
  delayMicroseconds(5);
  digitalWrite(Trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigpin, LOW);

  long duration = pulseIn(Echopin, HIGH, ECHO_TIMEOUT_US);
  lastTrigUs = micros();
  return (duration / 2) / 29.1;
}

void ServoWrite(int deg)
{
  int v = deg + servoTrimDeg;
  if (v < 0) v = 0;
  if (v > 180) v = 180;
  servo.write(v);
}

// 按角度差动态等待 (SG90 约 2ms/度, 含余量)
void servoTo(int absDeg)
{
  int target = absDeg;
  if (target < 0)   target = 0;
  if (target > 180) target = 180;

  int delta = target - curServoDeg;
  if (delta < 0) delta = -delta;

  ServoWrite(absDeg);
  curServoDeg = absDeg;
  delay(80 + delta * 2);
}

// 前方测距: 连测 3 次取中位数。
// 近距提前收工: 连续两次都 < stopDistanceCm 就立刻返回 (取更近的),
// 省下 60ms 的第三次回波 —— 那 60ms 在刹车时很关键。
int measureFront()
{
  int v[3];
  int n = 0;

  for (int i = 0; i < 3; i++) {
    int d = HC_SR04();
    if (d > 0) {
      v[n++] = d;
      if (n == 2 && v[0] < stopDistanceCm && v[1] < stopDistanceCm) {
        return (v[0] < v[1]) ? v[0] : v[1];
      }
    }
  }

  if (n == 0) return 0;
  if (n == 1) return v[0];
  if (n == 2) return (v[0] + v[1]) / 2;

  int a = v[0], b = v[1], c = v[2];
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// 左右各看一眼: 舵机直接摆到正侧方各测一次。
//
// 原来在 60 度和 75 度各测一次再取最大, 理由是"单点可能打在挡板接缝上"。
// 但代价是每个方向要摆两次, 一次避障就是 4 次摆动。
// 现在一个方向只摆一次, 快一倍; 真遇到缝隙读数异常,
// 后面的"转完再测一次, 不行就再转" 会兜住。
int lookSide(bool left)
{
  // 不能直接用 90 度: 舵机 90±90 已经到 0/180 机械极限,
  // 叠加 servoTrimDeg 后一侧会被夹住, 左右视野不对称。
  // 留出中位修正的量, 保证两侧都真的摆到底。
  int rel = sideLookDeg;
  int lim = 90 - abs(servoTrimDeg);
  if (rel > lim) rel = lim;

  servoTo(left ? (90 - rel) : (90 + rel));
  int d = HC_SR04();
  servoTo(90);
  return d;
}

// ---------------- 避障 ----------------
// 只有一条路径: 看左右 -> 选宽的一边 -> 转 90 -> 还是挡就再转。
// 没有扇扫, 没有内部循环, 没有斜向探测。
void avoidObstacle()
{
  Serial.println("avoid: look sides");

  int L = lookSide(true);
  int R = lookSide(false);

  bool goLeft = (L >= R);

  Serial.print("L="); Serial.print(L);
  Serial.print(" R="); Serial.print(R);
  Serial.println(goLeft ? " -> LEFT" : " -> RIGHT");

  blinkN(goLeft ? 1 : 2, 100, 100);

  // 转向前退一点, 给转弯让出空间 (原地转向时车身四角扫过的半径最大)
  Serial.println("back up");
  Backward();
  delay(backupMs);
  Stop();
  delay(150);

  int front = 0;
  int steps = 0;

  for (int i = 0; i < maxTurnSteps; i++) {
    Serial.print("turn 90 #"); Serial.println(i + 1);
    turnByDeg(goLeft, 90);
    steps++;

    front = measureFront();
    Serial.print("front after turn = "); Serial.println(front);

    if (front <= 0) continue;         // 读不到, 再转一次看看
    if (front >= clearCm) break;      // 通了, 收工
  }

  // 转了几次还是贴得很近: 说明在死角里, 再退出来一点
  if (front > 0 && front < emergencyCm) {
    Serial.println("still boxed -> back out more");
    Backward();
    delay(400);
    Stop();
    delay(200);
  }

  logEvent(L, R, front, goLeft, steps, 0, 0);

  if (front >= clearCm) blinkN(1, 400, 0);   // 长亮: 通了
  else                  blinkN(5, 60, 60);   // 快闪: 还是窄

  delay(200);
  digitalWrite(LED_PIN, LOW);

  escapeUntil = millis() + escapeMs;
}

// ---------------- 单轮点动测试 ----------------
void startTest(int n)
{
  Stop();
  testNum = n;
  testUntil = millis() + 8000;
  holdStop = true;
  Serial.print("TEST M"); Serial.print(n);
  Serial.println(" (8s, x=stop, g=resume)");

  Speed(0);
  if (n == 1) { motor1.setSpeed(255); motor1.run(FORWARD); }
  if (n == 2) { motor2.setSpeed(255); motor2.run(FORWARD); }
  if (n == 3) { motor3.setSpeed(255); motor3.run(FORWARD); }
  if (n == 4) { motor4.setSpeed(255); motor4.run(FORWARD); }
}

// ---------------- 串口 ----------------
void handleSerial()
{
  while (Serial.available()) {
    char c = Serial.read();

    if (c >= '1' && c <= '4') {
      startTest(c - '0');
    } else if (c == 'x') {
      testNum = 0;
      Stop();
      escapeUntil = 0;
      holdStop = true;
      Serial.println("STOP");
    } else if (c == 'g') {
      testNum = 0;
      holdStop = false;
      isMoving = false;
      escapeUntil = 0;
      Serial.println("GO");
    } else if (c == 'd') {
      testNum = 0;
      Stop();
      holdStop = true;
      dumpLog();
    } else if (c == 'c') {
      // 只清运行记录, 保留舵机中位
      for (int i = 0; i < TRIM_ADDR; i++) EEPROM.update(i, 0xFF);
      Serial.println("log cleared (servo trim kept)");
    } else if (c == '+' || c == '=') {
      servoTrimDeg++;
      if (servoTrimDeg > 90) servoTrimDeg = 90;
      ServoWrite(90);
      saveTrim();
    } else if (c == '-' || c == '_') {
      servoTrimDeg--;
      if (servoTrimDeg < -90) servoTrimDeg = -90;
      ServoWrite(90);
      saveTrim();
    } else if (c == 'k') {
      Serial.println("sweep center/left/right");
      ServoWrite(90);  delay(500);
      ServoWrite(0);   delay(700);
      ServoWrite(180); delay(700);
      ServoWrite(90);  delay(500);
    }
  }
}

// ---------------- 主流程 ----------------
void setup()
{
  Serial.begin(9600);
  loadTrim();

  Speed(0);
  pinMode(Trigpin, OUTPUT);
  pinMode(Echopin, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  servo.attach(10);
  ServoWrite(90);
  delay(1000);

  // 横幅必须打印"真正写进电机的占空比"。
  // 之前写的是 baseSpeed + trimRight, 但改按比例配平后 trimRight 恒为 0,
  // 横幅一直显示 L=180 R=180 (看起来没配平), 实际 R 是 180*1.23=221。
  // 调试时被这个假数据误导过, 所以改成按 Speed() 同样的算法算出来。
  int lOut = baseSpeed + trimLeft;
  int rOut = baseSpeed + (int)((long)baseSpeed * trimPct / 100) + trimRight;
  if (lOut > 255) lOut = 255;
  if (rOut > 255) rOut = 255;

  Serial.print("car ready  base="); Serial.print(baseSpeed);
  Serial.print(" L="); Serial.print(lOut);
  Serial.print(" R="); Serial.print(rOut);
  Serial.print(" trimPct="); Serial.print(trimPct);
  Serial.print(" msPer90="); Serial.print(msPer90Deg);
  Serial.print(" servo="); Serial.println(servoTrimDeg);
  Serial.println("cmd: 1-4 test / x stop / g go / d dump / c clear / +/- trim / k sweep");
}

void loop()
{
  handleSerial();

  if (testNum) {
    if ((long)(millis() - testUntil) >= 0) {
      testNum = 0;
      Stop();
      Serial.println("TEST done (halted, send g to resume)");
    }
    return;
  }

  if (holdStop) return;

  long distance = measureFront();
  Serial.print("Distance: ");
  Serial.println(distance);

  // 一个回波都没有: 不能当成"前方通畅"。慢慢试探, 别全速冲。
  if (distance <= 0) {
    Serial.println("no echo -> creep");
    StartMove(creepSpeed, 0);
    return;
  }

  // 极近距离: 无条件下先停再避障
  if (distance <= emergencyCm) {
    Serial.println("EMERGENCY");
    HardStop();
    avoidObstacle();
    return;
  }

  // 刚转完弯的脱离段: 只爬行, 不测距不判断, 免得在墙角里又触发一次避障
  if (escapeUntil && (long)(millis() - escapeUntil) < 0) {
    StartMove(creepSpeed, 0);
    return;
  }
  escapeUntil = 0;

  if (distance < stopDistanceCm) {
    Serial.println("Obstacle");
    HardStop();
    avoidObstacle();
    return;
  }

  // 通行: 近了降速, 远了全速
  StartMove(distance < slowDistanceCm ? creepSpeed : baseSpeed, distance);
}
