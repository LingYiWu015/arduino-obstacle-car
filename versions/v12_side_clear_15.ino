#include <Servo.h>
#include <AFMotor.h>
#include <EEPROM.h>

#define Trigpin A0
#define Echopin A1
#define LED_PIN 13          // 板载 LED, 用来在没串口时报告状态

Servo servo;
AF_DCMotor motor1(1);   // 实测: 左前
AF_DCMotor motor2(2);   // 实测: 左后
AF_DCMotor motor3(3);   // 实测: 右后
AF_DCMotor motor4(4);   // 实测: 右前

// ---------------- 参数 ----------------
const int baseSpeed = 100;         // 直行速度
const int kickSpeed = 255;         // 起步助推速度
const int kickMs    = 200;         // 起步助推时长 (太长会窜车, 实测 200 合适)

// ===== 直行配平 (解决越走越偏) =====
// 没有编码器/陀螺仪就无法测出真实航向, 只能靠两侧配平把"偏航速率"压到最小。
// 车往右偏 = 左侧比右侧快 -> 给 trimLeft 负值 (左侧降速)
// 车往左偏 = 右侧比左侧快 -> 给 trimLeft 正值 (左侧提速)
// 先每次 ±5 试, 找到"基本不偏"的值再以 ±2 精调。
// 实测: 车往右偏, 按实车判断右轮偏弱, 所以给 trimRight 正值 (右轮提速)。
// 实测: 1.5 米向右偏 0.25 米, 航向偏约 9.5 度 (每米 6.3 度)。
// 18 压不住, 加到 30。还偏就继续加, 开始往左偏就退回到临界点的一半。
const int trimLeft  = 0;
const int trimRight = 30;

// 转弯: 整个转弯过程用同一个速度, 不再中途变速,
// 这样"转多久 -> 转多少度"才是线性的。
// 标定实测: 350ms @ 速度150 转了约 52 度; 按比例放大到 650ms 约 90 度。
// 目标 85 度: 650 * 85/90 = 614, 取 615。
// 注意: 这个值只负责"转到大概 90 度", 差几度没关系, 后面瞄准会修掉。
const int turnSpeed  = 150;
const int msPer90Deg = 615;

// 安全距离。
// 实测: 28 太大。螺旋最内圈那段腿本来就短 (可能只有 20cm 出头),
// 一转弯前方就低于 28, 立刻又触发避障, 于是原地转了又转, 走不出直线。
// 降下来: 刹车实际只用掉几厘米 (baseSpeed=100 不快, 测距+停车 < 5cm),
// 真正的兜底交给 emergencyCm。
// 数值在下面 "通道几何" 一节由 sideClearCm 派生。
const int creepSpeed = 70;         // 贴墙前进 / 转过弯蹭出墙角用的慢速
const int escapeMs   = 400;        // 转过弯后强制前进的时长 (跳出墙角)
const int creepMinCm = 12;         // 前方小于这个值绝不往前开

// ===== 直角螺旋赛道专用转向参数 =====
// 赛道是正方形螺旋, 每个转角都是 90 度, 不存在"半个直角"。
//
// 关键认识: 不能再拿"转了多少度"当判据。
// 螺旋通道约 2 倍车宽, 角度再准, 也没法保证车头老老实实对正通道;
// 而且哪怕只差 1 度, 车角也可能刚好卡在墙上。
// 所以转弯改成两段: 先按时间粗转 90 度, 再用左右测距"瞄准"修正。
//
// 为什么要瞄准: 通道两侧墙就是天然参照物。
// 车头对正时, 左 45 度 和 右 45 度的读数接近; 偏了则一边明显更近。
// 这个差值比时间标定可靠, 而且能自动吃掉直行累积的偏航。

// 转弯前先倒一点, 拉开与墙的距离 (约 2.5cm)
// 没有后向传感器, 倒得越少越安全; 75ms 够脱开障碍物就行。
const int backupMs = 75;

// ---- 瞄准 ----
// ===== 基础模型 =====
// 把赛道抽象成: 一条笔直走廊, 两侧平行墙。
//
// 状态量只有两个:
//   h  = 车头相对走廊轴线的偏差角 (度, 正=偏右)
//   D  = 走廊是否"通" (存在某个方向能看到 > tightAllCm 的开阔处)
//
// 观测: 扇扫一圈, 得到距离剖面 d(θ)。
//   走廊存在时, d(θ) 在 θ = h 附近取最大值 (沿轴线看得最远)。
//   所以  argmax d(θ) 就是 h 的估计值。
//   全部 d(θ) < tightAllCm 时 D = 死胡同。
//
// 控制律:
//   1) 有 h 估计 -> 按 h 比例转 (单次限幅 turnMaxDeg), 而不是固定步长
//   2) 死胡同   -> 沿同方向再转 90 度重扫 (最内圈掉头就是这样走出来)
//
// 为什么这样能解决 45 度:
//   扇扫范围扩到 ±75 度, 能看到侧后方的出口;
//   比例修正允许单次转 45 度, 多轮累计可达 90 + 45*3 = 225 度,
//   足够覆盖最内圈需要的 180 度掉头。

// 扇扫角度点 (相对车头)。
// 中位修正 +30 后实际行程: 90-60+30 = 60; 90+60+30 = 180 (正好到限位)。
// 所以右端取 +55 留 5 度余量, 避免舵机顶着限位啸叫、读数失真。
const int scanAngles[] = { -60, -48, -36, -24, -12, 0, 12, 24, 36, 48, 55 };
const int scanCount    = 11;

const int aimDeadbandDeg = 14;  // |h| 小于这个就算对正 (必须 > 扫读步长 12, 否则永远修不到)
const int turnMaxDeg     = 45;  // 单次修正限幅, 防暴冲
const int maxAims        = 3;   // 每轮最多修正几次
const int maxExtraTurns  = 2;   // 死胡同最多再转几个 90 度 (2 次 = 掉头)

// ===== 通道几何: 以"车体单侧余量"为基准 =====
// 实测要求: 左右各留 15cm 才敢往前走。
// 探头装在车头, 朝左右各 55 度时测到的是"斜前方的墙面距离",
// 不是严格的横向余量, 所以留一点保守折算系数。
// 好处: 所有判定阈值都从这一个数派生, 以后改余量只改一行。
const int sideClearCm = 15;                    // 车体单侧所需余量
const int sideProxyCm = sideClearCm + 5;       // 斜扫读数的近似门限 (20cm)

// 左右两个方向都够宽, 且差值很小 -> 车在通道中间, 随便走
// 只有一侧够宽 -> 朝那一侧走
const int dirDeadbandCm = 8;

// ---- 判定 ----
const int clearCm = sideClearCm * 2;   // 前方 >= 30cm (两侧各 15) 才算这条通道能走
const int stopDistanceCm = sideClearCm + 1;  // 16: 前方低于它就避障

// 全方向都窄的判定: 扇扫最远处也不到这么远, 说明四周都是墙。
// 螺旋最内圈就是这样, 此时"哪边最远"纯属噪声, 修正是白转, 直接放弃瞄准。
const int tightAllCm = 40;

// 接近刹车线: 只有真贴到这么近才推翻"内圈直行"的判断
const int emergencyCm = 10;

const int servoMoveMs  = 260;   // 舵机转到位并稳定下来 (SG90 走 90 度约 150ms, 留余量)
const int turnBalanceCm = 25;   // 交替修正阈值 (逻辑已注释停用)

// 舵机中位修正 (已标定)
// 这个偏差来自舵盘花键错位 + 探头支架角度, 是机械误差, 只能靠软件平掉。
// 标定过程: -2 反向 / +5 +10 +17 +21 都还偏右 / 清零后偏得更多 / 30 略过 -> 26 正好。
// 26 是最终值, 探头光轴已与车身中线对齐。不要再随意改动。
// 需要时串口发 + / - 每次 1 度, 存进 EEPROM, 下次开机自动读回。
int servoTrimDeg = 26;

// ---------------- 状态 ----------------
bool isMoving = false;             // 用于判断是否要起步助推
int  testNum = 0;                  // 单轮点动编号, 0 = 不在测试
unsigned long testUntil = 0;
bool holdStop = false;             // 测试/急停后保持停车, 直到串口发 g
bool lastWentLeft = false;         // 上次往哪边转, 用于交替避障
bool tightAllRound = false;        // 上一轮避障时四周都窄 (螺旋最内圈)
unsigned long escapeUntil = 0;     // 转过弯后强制前进到这个时刻, 用来跳出墙角

// ---------------- 基础动作 ----------------
// 直行: 侧速度 = 基础值 + 该侧配平, 左右独立, 用来抵消偏航。
void Speed(int s)
{
  int l = s + trimLeft;    // M1 左前, M2 左后
  int r = s + trimRight;   // M3 右后, M4 右前
  if (l < 0) l = 0; if (l > 255) l = 255;
  if (r < 0) r = 0; if (r > 255) r = 255;

  motor1.setSpeed(l);
  motor2.setSpeed(l);
  motor3.setSpeed(r);
  motor4.setSpeed(r);
}

// 转弯: 不加直行配平, 否则左右转幅度会不对称。
void TurnSpeed(int s)
{
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  motor1.setSpeed(s);
  motor2.setSpeed(s);
  motor3.setSpeed(s);
  motor4.setSpeed(s);
}

// 刹车: 只能靠 RELEASE (L293D 两输入全低 = 电机短接 = 动态制动)
void Stop()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  isMoving = false;
}

// 刹车 + 停稳。转弯/避障收尾统一用它, 避免惯性继续滑。
void Brake()
{
  Stop();
  delay(100);
}

void Forward()
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void TurnLeft()      // 左侧前进, 右侧后退
{
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  TurnSpeed(turnSpeed);
}

void TurnRight()     // 左侧后退, 右侧前进
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  TurnSpeed(turnSpeed);
}

void ServoWrite(int deg)
{
  // 角度先夹到 0..180 再写, 避免超量程时舵机持续堵转啸叫
  int v = deg + servoTrimDeg;
  if (v < 0) v = 0;
  if (v > 180) v = 180;
  servo.write(v);
}

// 保存/读取舵机中位修正
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

// ---------------- 倒车 / 微调 ----------------
// 倒车: 脱离障碍物用。退一点再转, 转弯半径更小, 也更不容易蹭墙。
void Backward()
{
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  TurnSpeed(turnSpeed);
}

// 按角度转: 角度 -> 时间, 统一换算
void turnByDeg(bool left, int deg)
{
  int ms = (int)((long)msPer90Deg * deg / 90);
  if (left) TurnLeft();
  else      TurnRight();
  delay(ms);
  Brake();
}

// ---------------- 板载反馈 ----------------
// 没无线串口时, 靠 D13 灯看它做了什么:
//   1 短闪   = 往左转      2 短闪 = 往右转
//   之后 N 闪 = 瞄准修正了 N 次
//   最后 1 长闪 = 前方通,  快速 5 闪 = 前方还是窄
void blinkN(int n, int onMs, int offMs)
{
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(onMs);
    digitalWrite(LED_PIN, LOW);  delay(offMs);
  }
}

// ---------------- EEPROM 记录 ----------------
// UNO 有 1KB EEPROM, 存最近 127 次避障的读数, 跑完插 USB 读出来。
// 记录格式 (8 字节): A5 | 左 | 右 | 前方 | 方向 | 修正次数 | 左修 | 右修
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
  Serial.println("L R front dir fixes aimL aimR");

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
// 超时设成 25000us:
//   pulseIn 默认超时是 1 秒! 读不到回波 (斜墙、吸音面、别的车干扰) 时会死等 1 秒。
//   避障一次要扫十几个点, 有几个读不到就是十几秒的纯等待, 这是"检查太慢"的主因。
//   25ms 对应约 4 米, 远超赛道尺度, 读不到就该立刻放弃而不是等。
#define ECHO_TIMEOUT_US 25000

long HC_SR04()
{
  // 两次触发间隔 >=60ms, 放在触发之前 (放中间会错过回波)
  delay(60);
  digitalWrite(Trigpin, LOW);
  delayMicroseconds(5);
  digitalWrite(Trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigpin, LOW);
  long duration = pulseIn(Echopin, HIGH, ECHO_TIMEOUT_US);
  return (duration / 2) / 29.1;
}

// 转到相对车头 deg 度 (负=左, 正=右) 测一次, 测完自动回中
int lookAtRel(int rel)
{
  ServoWrite(90 + rel);
  delay(servoMoveMs);
  int d = HC_SR04();
  ServoWrite(90);
  delay(servoMoveMs);
  return d;
}

// 从当前角度连续测几个点, 不再每次都回中。
// 扇扫用这个: 5 个点从最左扫到最右, 舵机单向走, 比"出去再回来"省一半时间。
void sweepInto(int *buf, int n)
{
  for (int i = 0; i < n; i++) {
    ServoWrite(90 + scanAngles[i]);
    delay(servoMoveMs);
    buf[i] = HC_SR04();
  }
  ServoWrite(90);
  delay(servoMoveMs);
}

// 多测几次取中位数。原来取"第一个非 0", 一次虚假远回波就会让它放心撞上去。
int measureFront()
{
  int v[3];
  int n = 0;
  for (int i = 0; i < 3; i++) {
    int d = HC_SR04();
    if (d > 0) v[n++] = d;
  }
  if (n == 0) return 0;
  if (n == 1) return v[0];
  if (n == 2) return (v[0] + v[1]) / 2;
  // 三个都有: 取中位数, 剔除最大最小
  int a = v[0], b = v[1], c = v[2];
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

// 扇扫 -> 估计走廊轴线方向。
//   *bestDist 写回最远距离
//   *open     写回: 是否存在开阔方向 (false = 四周都是墙, 死胡同)
// 返回值 = 车头相对走廊轴线的偏差角 h (度, 正=轴线在右边)
int sweepBest(int *bestDist, bool *open)
{
  int buf[scanCount];
  sweepInto(buf, scanCount);       // 单向扫一遍, 比来回跑快一半

  int best = 0;
  int bestD = 0;          // 0 = 还没有有效读数
  bool any = false;

  for (int i = 0; i < scanCount; i++) {
    int rel = scanAngles[i];
    int d = buf[i];

    Serial.print("  scan "); Serial.print(rel);
    Serial.print("deg = "); Serial.print(d); Serial.println("cm");

    // 0 表示没回波 (超量程或吸音面), 不能当"最远"用
    if (d <= 0) continue;

    any = true;
    if (d > bestD) {
      bestD = d;
      best = rel;
    }
  }

  *bestDist = bestD;

  if (!any) {             // 整轮都没读数, 无从判断, 当对正
    *open = false;
    return 0;
  }

  if (bestD < tightAllCm) {   // 四周都是墙 -> 死胡同
    *open = false;
    return best;
  }

  *open = true;
  return best;
}

// ---------------- 避障 ----------------
void checkDirection()
{
  Serial.println("scan...");

  // 左右各扫 55 度。必须用相对角度而不是硬写 0/180:
  // servoTrimDeg = 26 时, ServoWrite(180) 实际写 206 -> 被夹到 180,
  // 左扫变成 -64 度而右扫只有 +90 度, 两侧视野严重不对称,
  // 会让"哪边更远"的判断系统性偏向一边。用相对角就能保证对称。
  int distanceLeft  = lookAtRel(-55);   // 车身左侧
  int distanceRight = lookAtRel(55);    // 车身右侧

  Serial.print("Left: ");  Serial.print(distanceLeft);
  Serial.print("cm, Right: "); Serial.println(distanceRight);

  // 左右判据改成"这一侧够不够 15cm 余量", 而不是简单比大小。
  bool leftOk  = (distanceLeft  >= sideProxyCm);
  bool rightOk = (distanceRight >= sideProxyCm);

  bool goLeft;
  if (leftOk != rightOk) {
    // 只有一侧够宽: 走那一侧。这是最有把握的情况。
    goLeft = leftOk;
    Serial.println(leftOk ? "only left clear" : "only right clear");
  } else if (leftOk && rightOk) {
    // 两侧都够宽: 都在通道内, 走哪边都行, 取更远的一边让转弯更从容
    goLeft = (distanceLeft > distanceRight);
    Serial.println("both clear -> wider");
  } else {
    // 两侧都不够宽: 已经贴墙了, 靠噪声选边没有意义, 沿用上次的方向
    goLeft = lastWentLeft;
    Serial.println("both tight -> keep last dir");
  }

  // 两侧读数接近时不要被 1~2cm 的抖动翻来覆去, 沿用上次方向
  if (abs(distanceLeft - distanceRight) < dirDeadbandCm) {
    goLeft = lastWentLeft;
    Serial.println("readings close -> keep last dir");
  }

  // 交替转向修正: 已停用。保留代码方便以后打开。
  // 逻辑: 左右差距小于 turnBalanceCm 时强制和上次反向, 抵消固定偏航趋势。
  // 问题: 阈值较小时效果不明显, 阈值较大时可能撞窄的一侧。
  // if (abs(distanceLeft - distanceRight) < turnBalanceCm) {
  //   goLeft = !lastWentLeft;
  //   Serial.println("balanced -> alternate");
  // }
  lastWentLeft = goLeft;

  // ---- 第一步: 先倒一点, 拉开与墙的距离 ----
  // 原地转向时车身四个角扫过的半径最大, 贴墙太近容易蹭。
  // 没有后向传感器, 只能靠"退得很少"(约 5cm) 保证安全。
  Serial.println("back up");
  Backward();
  delay(backupMs);
  Stop();
  delay(150);

  // ---- 第二步: 粗转 90 度 ----
  Serial.print("Turn ");
  Serial.println(goLeft ? "Left" : "Right");
  blinkN(goLeft ? 1 : 2, 100, 100);

  turnByDeg(goLeft, 90);

  // ---- 第三步: 瞄准 (闭环) ----
  // 按基础模型做比例修正:
  //   扇扫得到走廊轴线方向 h -> 车头就朝 h 转, 一次转到位, 单次限幅 turnMaxDeg。
  //   如果四周全是墙 (死胡同), 就沿当前方向再转 90 度重新扫,
  //   因为出口可能已经跑到车的侧后方了。
  // 累计能力: 粗转 90 + 修正 45*3 + 补转 90*2 = 315 度, 足够任何掉头。
  int fixes = 0;
  int aimsLeft = 0;
  int aimsRight = 0;
  int extraTurns = 0;
  int best = 0;
  int bestDist = 0;
  bool open = false;

  while (true) {
    best = sweepBest(&bestDist, &open);

    Serial.print("aim h="); Serial.print(best);
    Serial.print("deg dist="); Serial.print(bestDist);
    Serial.println(open ? " (open)" : " (boxed)");

    if (!open) {
      // 整个扇扫一个有效回波都没有 -> 传感器瞎了, 不是死胡同, 别乱转
      if (bestDist <= 0) {
        Serial.println("no echo at all -> stop scanning");
        break;
      }

      // 死胡同: 四周都不开阔, 出口不在前方视野内
      if (extraTurns >= maxExtraTurns) {
        Serial.println("boxed -> give up, creep forward");
        tightAllRound = true;
        break;
      }
      Serial.println("boxed -> turn 90");
      turnByDeg(goLeft, 90);
      extraTurns++;
      fixes++;
      continue;
    }

    if (abs(best) <= aimDeadbandDeg) {
      Serial.println("aimed");
      break;
    }

    // 比例修正: 转多少由 h 决定, 不拍脑袋给固定步长
    int step = abs(best);
    if (step > turnMaxDeg) step = turnMaxDeg;
    bool fixLeft = (best < 0);

    Serial.print("aim fix "); Serial.print(fixLeft ? "L" : "R");
    Serial.print(' '); Serial.println(step);

    turnByDeg(fixLeft, step);
    if (fixLeft) aimsLeft++; else aimsRight++;
    fixes++;

    if (fixes >= maxAims + maxExtraTurns) break;   // 总量兜底
  }

  // 修正次数用灯码报出来, 跑的时候能看出它有没有来回抖
  blinkN(fixes % 5, 80, 80);

  // ---- 第四步: 验收, 并记录 ----
  int front = measureFront();
  Serial.print("front: "); Serial.print(front); Serial.println("cm");

  logEvent(distanceLeft, distanceRight, front, goLeft, fixes, aimsLeft, aimsRight);

  if (front >= clearCm) {
    Serial.println("path clear");
    blinkN(1, 400, 0);          // 长亮一下: 通
  } else {
    Serial.println("still tight");
    blinkN(5, 60, 60);          // 快闪 5 下: 还是窄
  }

  delay(200);
  digitalWrite(LED_PIN, LOW);
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
  Serial.print("car ready  base="); Serial.print(baseSpeed);
  Serial.print(" L="); Serial.print(baseSpeed + trimLeft);
  Serial.print(" R="); Serial.print(baseSpeed + trimRight);
  Serial.print(" msPer90="); Serial.print(msPer90Deg);
  Serial.print(" servo="); Serial.println(servoTrimDeg);
  Serial.println("cmd: 1-4 test / x stop / g go / d dump / c clear / +/- servo trim / k sweep");
}

void loop()
{
  // 串口: 1-4 单轮点动 / x 停 / g 继续
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '4') {
      startTest(c - '0');
    } else if (c == 'x') {
      testNum = 0;
      Stop();
      holdStop = true;
      Serial.println("STOP");
    } else if (c == 'g') {
      testNum = 0;
      holdStop = false;
      isMoving = false;
      Serial.println("GO");
    } else if (c == 'd') {
      // 跑完插 USB 发 d, 把 EEPROM 里的运行记录打出来
      testNum = 0;
      Stop();
      holdStop = true;
      dumpLog();
    } else if (c == 'c') {
      // 只清运行记录, 不能顺手把舵机中位 (TRIM_ADDR 附近) 也擦掉,
      // 否则下次开机 loadTrim 读不到 magic, 中位会悄悄回落到编译默认值。
      for (int i = 0; i < TRIM_ADDR; i++) EEPROM.update(i, 0xFF);
      Serial.println("log cleared (servo trim kept)");
    } else if (c == '+' || c == '=') {
      // 在线调舵机中位: + 往左修 (值变大), 存 EEPROM
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
      // 手动扫一遍, 用来目视对中: 舵机会依次停在 0 90 180
      Serial.println("sweep center/left/right");
      ServoWrite(90);  delay(500);
      ServoWrite(0);   delay(700);
      ServoWrite(180); delay(700);
      ServoWrite(90);  delay(500);
    }
  }

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

  // ---- 转过弯后的脱离段 ----
  // 刚转完 90 度时车还贴在墙角里, 前方读数必然很短。
  // 如果这时按"前方窄就避障"处理, 它会在同一个角落转了又转, 永远走不出去。
  // 所以转完强制往前蹭一小段, 只保留 emergencyCm 这道硬刹车。
  if (escapeUntil && (long)(millis() - escapeUntil) < 0) {
    if (distance > 0 && distance >= emergencyCm) {
      Forward();
      if (!isMoving) { Speed(kickSpeed); delay(kickMs); isMoving = true; }
      Speed(creepSpeed);
      return;
    }
    escapeUntil = 0;              // 真贴墙了, 放弃脱离, 进入正常避障
  } else {
    escapeUntil = 0;
  }

  // 螺旋最内圈: 四面都是墙, 前方读数一直很短。
  // 这种情况靠前进穿过最内圈, 而不是原地反复修正。
  if (tightAllRound) {
    if (distance >= clearCm) {
      tightAllRound = false;      // 已经走出内圈, 恢复正常判断
    } else if (distance > 0 && distance >= emergencyCm) {
      Forward();
      if (!isMoving) { Speed(kickSpeed); delay(kickMs); isMoving = true; }
      Speed(creepSpeed);
      return;
    }
  }

  if (distance < stopDistanceCm && distance != 0) {
    Serial.println("Obstacle!");
    Stop();
    delay(200);
    tightAllRound = false;
    escapeUntil = 0;
    checkDirection();
    // 转完立刻给一段脱离时间, 别让它在墙角里反复触发
    escapeUntil = millis() + escapeMs;
  } else {
    Forward();
    if (!isMoving) {          // 从静止起步, 四轮一起冲一下
      Speed(kickSpeed);
      delay(kickMs);
      isMoving = true;
    }
    Speed(baseSpeed);
  }
}
