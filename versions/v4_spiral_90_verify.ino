#include <Servo.h>
#include <AFMotor.h>

#define Trigpin A0
#define Echopin A1

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
// 目标 85 度: 650 * 85/90 = 614, 取 615。转多了减, 转少了加。
const int turnSpeed = 150;
const int turnTime  = 615;

const int stopDistanceCm = 28;     // 前方多少 cm 开始避障 (实测 20 太贴, 28 更稳)

// ===== 直角螺旋赛道专用转向参数 =====
// 赛道是正方形螺旋, 每个转角都是 90 度, 不存在"半个直角"。
// 所以不再按距离分档, 统一转 90 度, 转不够再补 45 度。

// 转弯前先倒一点, 拉开与墙的距离 (约 5cm)
const int backupMs = 150;

// 第一次转 90 度用这个时长 (按 turnTime 标定值)
// 转弯后验收: 前方 >= passDistanceCm 就算转够了
const int passDistanceCm = 55;

// 补转步长 (度), 每次补这么多
const int fixStepDeg = 45;
// 最多补几次
const int maxFixes = 2;

const int turnBalanceCm = 25;   // 交替修正阈值 (逻辑已注释停用)
const int servoTrimDeg   = 10;     // 舵机中位修正 (实测 +10 才正)

// ---------------- 状态 ----------------
bool isMoving = false;             // 用于判断是否要起步助推
int  testNum = 0;                  // 单轮点动编号, 0 = 不在测试
unsigned long testUntil = 0;
bool holdStop = false;             // 测试/急停后保持停车, 直到串口发 g
bool lastWentLeft = false;         // 上次往哪边转, 用于交替避障

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
  int v = deg + servoTrimDeg;
  if (v < 0) v = 0;
  if (v > 180) v = 180;
  servo.write(v);
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

// 转弯 + 时间 (ms), 单位统一, 便于按角度换算
void turnFor(bool left, int ms)
{
  if (left) TurnLeft();
  else      TurnRight();
  delay(ms);
  Stop();
}

// ---------------- 测距 ----------------
long HC_SR04()
{
  // 两次触发间隔 >=60ms, 放在触发之前 (放中间会错过回波)
  delay(60);
  digitalWrite(Trigpin, LOW);
  delayMicroseconds(5);
  digitalWrite(Trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigpin, LOW);
  long duration = pulseIn(Echopin, HIGH);
  return (duration / 2) / 29.1;
}

// ---------------- 避障 ----------------
void checkDirection()
{
  Serial.println("scan...");

  ServoWrite(0);                 // 看左边
  delay(600);
  int distanceLeft = HC_SR04();

  ServoWrite(180);               // 看右边
  delay(600);
  int distanceRight = HC_SR04();

  ServoWrite(90);                // 回中
  delay(400);

  Serial.print("Left: ");  Serial.print(distanceLeft);
  Serial.print("cm, Right: "); Serial.println(distanceRight);

  bool goLeft = (distanceLeft > distanceRight);

  // 交替转向修正: 已停用。保留代码方便以后打开。
  // 逻辑: 左右差距小于 turnBalanceCm 时强制和上次反向, 抵消固定偏航趋势。
  // 问题: 阈值较小时效果不明显, 阈值较大时可能撞窄的一侧。
  // if (abs(distanceLeft - distanceRight) < turnBalanceCm) {
  //   goLeft = !lastWentLeft;
  //   Serial.println("balanced -> alternate");
  // }
  lastWentLeft = goLeft;

  // ===== 直角螺旋: 统一转 90 度 =====
  // 赛道是正方形螺旋, 每个转角都是直角, 不需要按距离分档。
  // 转向用固定 90 度, 转完再验收, 不够就补 45 度。
  int ms = turnTime;

  // ---- 第一步: 先倒一点, 拉开与墙的距离 ----
  // 原地转向时车身四个角扫过的半径最大, 贴墙太近容易蹭。
  // 没有后向传感器, 只能靠"退得很少"(约 5cm) 保证安全。
  Serial.println("back up");
  Backward();
  delay(backupMs);
  Stop();
  delay(150);

  // ---- 第二步: 转 90 度 ----
  if (goLeft) {
    Serial.println("Turn Left 90");
    TurnLeft();
  } else {
    Serial.println("Turn Right 90");
    TurnRight();
  }
  delay(ms);
  Stop();
  delay(300);

  // ---- 第三步: 验收 ----
  // 转完直接测前方: 够宽就收工, 不够就补 45 度, 最多 maxFixes 次。
  // 实际转角由测距闭环决定, 不靠时间推算, 也不怕固定偏航误差。
  for (int fix = 0; fix < maxFixes; fix++) {
    int front = HC_SR04();
    Serial.print("after turn: "); Serial.print(front);
    Serial.println("cm");

    if (front >= passDistanceCm) {
      Serial.println("path clear");
      break;
    }

    Serial.print("tight -> +"); Serial.println(fixStepDeg);
    if (goLeft) TurnLeft(); else TurnRight();
    delay((int)((long)turnTime * fixStepDeg / 90));
    Stop();
    delay(250);
  }
  delay(200);
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
  Speed(0);
  pinMode(Trigpin, OUTPUT);
  pinMode(Echopin, INPUT);
  servo.attach(10);
  ServoWrite(90);
  delay(1000);
  Serial.print("car ready  base="); Serial.print(baseSpeed);
  Serial.print(" L="); Serial.print(baseSpeed + trimLeft);
  Serial.print(" R="); Serial.print(baseSpeed + trimRight);
  Serial.print(" turn="); Serial.print(turnTime);
  Serial.print(" servo="); Serial.println(servoTrimDeg);
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

  long distance = HC_SR04();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < stopDistanceCm && distance != 0) {
    Serial.println("Obstacle!");
    Stop();
    delay(200);
    checkDirection();
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
