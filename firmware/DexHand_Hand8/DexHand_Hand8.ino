/*
 * DexHand_Hand8.ino  (rev.4 — 2026-08-18)
 *
 * DexHand 손가락 4개 x 너클 2모터 = 8서보 통합 제어, 캘리브레이션, 동작 테스트 스케치
 * 대상: Arduino Mega ADK(Mega 2560 계열) + PCA9685 + 외부 5V SMPS
 *
 * ── rev.4 변경점 (ROS 2 / MoveIt GUI 연동을 위해) ─────────────────────
 *  1) fspan / yspan 상한을 200us -> 700us 로 확장. (2026-08-19: 1000us 로 재확장, 중립 1900 체계)
 *     3차 세션에서 실측한 여유(room)가 채널당 630~700us 인데 상한이 200us 로
 *     하드코딩되어 있어서 `fspan 450` 을 보내도 조용히 200 으로 잘렸다.
 *     파지에 필요한 350~500us 를 쓸 수 없던 원인이다.
 *  2) 고해상도 스트림 명령 `w` 추가 (-1000..1000, 0.1% 단위).
 *     PC 에서 IK 결과를 rad 단위로 보낼 때 기존 `v` 의 1% 해상도(= 700us 스팬에서 7us)
 *     로는 손끝이 눈에 띄게 계단처럼 움직인다.
 *     내부 명령값을 전부 per-mille(-1000..1000) 로 바꿨고,
 *     기존 `f`/`y`/`fa`/`ya`/`v` 는 -100..100 그대로 받아 x10 해서 쓴다. 하위호환된다.
 *  3) `room` 명령 (채널별 중립 기준 여유와 현재 span 의 clamp 여부).
 *  4) HARD_MIN_US/HARD_MAX_US 를 800/2200 으로, 기본 캘리브레이션을 3차 세션 실측값으로.
 *     EEPROM 매직도 0xDE8A0802 로 올려 구버전 레이아웃을 자동 무시한다.
 *  5) `ping` 명령: PC 드라이버가 링크 생존을 확인할 때 쓴다.
 *
 * ── 배선 ────────────────────────────────────────────────────────────
 *   Mega 5V      -> PCA9685 VCC          (로직 전원)
 *   Mega GND     -> PCA9685 GND          (외부 SMPS GND와 반드시 공통)
 *   Mega SDA(20) -> PCA9685 SDA
 *   Mega SCL(21) -> PCA9685 SCL
 *   Mega D7      -> PCA9685 OE           (HIGH = 출력 차단)
 *   5V 15A SMPS  -> PCA9685 V+ / GND     (서보 전원, Mega에서 뽑지 말 것)
 *
 * ── 채널 매핑 (DexHand 원본 명명 규칙) ──────────────────────────────
 *   CH0  FON  Fore(검지)   Outward   |  CH1  FII  Fore   Inward
 *   CH2  MON  Middle(중지) Outward   |  CH3  MIN  Middle Inward
 *   CH4  ROI  Ring(약지)   Outward   |  CH5  RII  Ring   Inward
 *   CH6  LON  Little(소지) Outward   |  CH7  LII  Little Inward
 *
 *   손가락 f (0=검지, 1=중지, 2=약지, 3=소지)
 *     Outward 서보 = f*2,  Inward 서보 = f*2+1
 *
 * ── 제어 모델 ───────────────────────────────────────────────────────
 *   손가락 1개 = 2 자유도 (flex = MCP 굽힘, yaw = 좌우 벌림)
 *     us = neutral + flexSign*flex*flexSpan/1000 + yawSign*yaw*yawSpan/1000
 *   (flex, yaw 는 내부적으로 -1000..1000)
 *
 * ── 안전 ────────────────────────────────────────────────────────────
 *   - 부팅 직후 항상 출력 차단 상태로 시작한다.
 *   - off, x, 빈 줄(엔터만)은 어느 모드에서나 비상 정지다.
 *   - 모든 이동은 속도 제한(us/s)이 걸린 논블로킹 램프로만 수행된다.
 *   - 스트리밍 모드는 워치독이 있다. 패킷이 끊기면 정지 후 출력 차단한다.
 *   - 최종 비상 수단은 언제나 서보 전원(V+)의 물리적 차단이다.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// ================= 하드웨어 설정 =================
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t OE_PIN = 7;                 // HIGH = 출력 차단
constexpr float   SERVO_FREQ_HZ = 50;
constexpr uint32_t PCA9685_OSC_HZ = 27000000; // 펄스폭이 어긋나면 25~27MHz에서 조정

constexpr uint8_t NUM_FINGERS = 4;
constexpr uint8_t SERVOS_PER_FINGER = 2;      // Outward, Inward
constexpr uint8_t NUM_SERVOS = NUM_FINGERS * SERVOS_PER_FINGER;  // 8

// 3차 세션에서 8채널 전부 800~2200us 구동을 실물로 확인했다.
constexpr int HARD_MIN_US = 800;   // ES3352 스펙 800~2200us (2026-08-19 실측: 750 이하는 끝단 고정)
constexpr int HARD_MAX_US = 2200;
constexpr int START_US = 1500;

// 명령 스케일. 내부 flex/yaw 는 -CMD_FULL..CMD_FULL
constexpr int CMD_FULL = 1000;

// span 상한. 실측 여유(room)의 최솟값이 630us 이므로 그보다 크게 잡되,
// fspan+yspan 이 room 을 넘으면 clamp 된다는 건 `room` 명령으로 확인한다.
constexpr int SPAN_MAX_US = 1000;

// 논블로킹 모션 엔진
constexpr uint8_t MOTION_TICK_MS = 10;        // 10ms 주기로 목표를 향해 이동
constexpr int SPEED_MIN = 50;                 // us/s
constexpr int SPEED_MAX = 3000;               // us/s

// 스트리밍 워치독
constexpr uint16_t STREAM_FREEZE_MS = 400;    // 이 시간 패킷 없으면 목표 동결
constexpr uint16_t STREAM_OFF_MS = 2000;      // 이 시간 패킷 없으면 출력 차단

constexpr uint8_t CMD_BUF_SIZE = 96;

// ================= 캘리브레이션 =================
struct ServoCal {
  int16_t minUs;      // 안전 하한 (jog 실측 후 min 기록)
  int16_t maxUs;      // 안전 상한
  int16_t neutralUs;  // 중립, 텐던 유격만 제거된 위치
  int8_t  flexSign;   // +1 / -1
  int8_t  yawSign;    // +1 / -1
};

struct CalStore {
  uint32_t magic;
  ServoCal cal[NUM_SERVOS];
  int16_t  flexSpanUs;
  int16_t  yawSpanUs;
  int16_t  speedUsPerSec;
};

constexpr uint32_t CAL_MAGIC = 0xDE8A0802UL;  // rev.3 이후 레이아웃

// 2026-08-19 텐던 장착 후 실측 (4손가락 전부 동일 패턴):
//   - 두 서보 모두 "낮은 us = 텐던 당김". 굽힘(flex+) = 둘 다 내려감 -> flexSign 전 채널 -1.
//   - 좌우(yaw) = 차동 -> Outward +1 / Inward -1.
//   - 중립 1900: 손가락이 곧게 펴지고 텐던이 막 팽팽한 지점. 굽힘은 한 방향(0..+)만 쓰므로
//     중립을 위쪽 끝 근처에 두어 아래로 1000us(1900->900)를 전부 굽힘 행정으로 쓴다.
//     ES3352 스펙 800~2200us, 750 이하는 끝단 고정(실측).
//   - 굽힘 시작 ~1500us, 최대 ~900us(검지 ~90°, 중지/약지는 더 얕음). 800에선 더 안 굽음.
ServoCal servoCal[NUM_SERVOS] = {
  // minUs, maxUs, neutralUs, flexSign, yawSign
  {800, 2200, 1900, -1, +1},  // 0 FON  검지 Outward
  {800, 2200, 1900, -1, -1},  // 1 FII  검지 Inward
  {800, 2200, 1900, -1, +1},  // 2 MON  중지 Outward
  {800, 2200, 1900, -1, -1},  // 3 MIN  중지 Inward
  {800, 2200, 1900, -1, +1},  // 4 ROI  약지 Outward
  {800, 2200, 1900, -1, -1},  // 5 RII  약지 Inward
  {800, 2200, 1900, -1, +1},  // 6 LON  소지 Outward
  {800, 2200, 1900, -1, -1},  // 7 LII  소지 Inward
};

// ROS 드라이버(servo_map.yaml)의 flex_span_us / yaw_span_us 와 반드시 같아야 한다.
int flexSpanUs = 950;   // 1900-950-150 = 800 = HARD_MIN: 최대 굽힘+최대 yaw 에서도 clamp 없음
int yawSpanUs  = 150;
int speedUsPerSec = 400;

const char SERVO_NAME[NUM_SERVOS][4] = {
  "FON", "FII", "MON", "MIN", "ROI", "RII", "LON", "LII"
};
const char FINGER_NAME[NUM_FINGERS][7] = {"Fore", "Middle", "Ring", "Little"};

// ================= 상태 =================
enum class Mode : uint8_t { Jog, Finger, Stream };
enum class Test : uint8_t { None, Sweep, Seq, Wave, Sine };

Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

Mode mode = Mode::Finger;
bool outputEnabled = false;
bool i2cOk = false;
uint8_t jogServo = 0;

int currentUs[NUM_SERVOS];
int targetUs[NUM_SERVOS];
int flexCmd[NUM_FINGERS];   // -1000..1000
int yawCmd[NUM_FINGERS];    // -1000..1000

unsigned long lastMotionMs = 0;
unsigned long lastPacketMs = 0;
bool streamFrozen = false;

char cmdBuf[CMD_BUF_SIZE];
uint8_t cmdLen = 0;

// ================= 포즈 테이블 =================
// flex, yaw 는 -100..100 (사람이 읽기 쉬운 단위). 적용할 때 x10 한다.
struct Pose {
  int8_t   flex[NUM_FINGERS];
  int8_t   yaw[NUM_FINGERS];
  uint16_t holdMs;
};

const Pose POSE_SEQUENTIAL[] PROGMEM = {
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 400},
  {{ 70,  0,  0,  0}, {0, 0, 0, 0}, 600},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 400},
  {{  0, 70,  0,  0}, {0, 0, 0, 0}, 600},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 400},
  {{  0,  0, 70,  0}, {0, 0, 0, 0}, 600},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 400},
  {{  0,  0,  0, 70}, {0, 0, 0, 0}, 600},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 400},
};

const Pose POSE_OPENCLOSE[] PROGMEM = {
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 500},
  {{ 80, 80, 80, 80}, {0, 0, 0, 0}, 700},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 500},
  {{-40,-40,-40,-40}, {0, 0, 0, 0}, 500},
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 500},
};

// t5 스프레드. 부호 주의: URDF FK 로 확인한 결과 +yaw 는 네 손끝을 전부 같은 쪽으로 민다.
// 벌리려면 검지와 소지가 서로 반대여야 한다.
const Pose POSE_SPREAD[] PROGMEM = {
  {{0, 0, 0, 0}, {   0,   0,   0,   0}, 500},
  {{0, 0, 0, 0}, { -80, -30,  30,  80}, 800},   // 벌리기
  {{0, 0, 0, 0}, {   0,   0,   0,   0}, 500},
  {{0, 0, 0, 0}, {  30,  10, -10, -30}, 800},   // 모으기 (충돌 여유가 좁아 작게)
  {{0, 0, 0, 0}, {   0,   0,   0,   0}, 500},
};

const Pose POSE_COUNT[] PROGMEM = {
  {{ 85, 85, 85, 85}, {0, 0, 0, 0}, 600},   // 주먹
  {{  0, 85, 85, 85}, {0, 0, 0, 0}, 800},   // 1
  {{  0,  0, 85, 85}, {0, 0, 0, 0}, 800},   // 2
  {{  0,  0,  0, 85}, {0, 0, 0, 0}, 800},   // 3
  {{  0,  0,  0,  0}, {0, 0, 0, 0}, 800},   // 4
  {{ 85, 85, 85, 85}, {0, 0, 0, 0}, 600},
};

struct SeqDef {
  const Pose* poses;
  uint8_t     count;
};

SeqDef activeSeq = {nullptr, 0};

Test activeTest = Test::None;
uint8_t testStep = 0;
uint8_t testRepeat = 1;
uint8_t testRepeatLeft = 0;
uint8_t testSubStep = 0;
unsigned long procTimerMs = 0;
uint16_t seqHoldMs = 0;
unsigned long seqHoldStartMs = 0;
bool testWaitingHold = false;
int defaultRepeats = 2;

// ================= 유틸 =================
uint8_t fingerOf(uint8_t servoIdx) { return servoIdx / SERVOS_PER_FINGER; }

void printServoName(uint8_t idx) { Serial.print(SERVO_NAME[idx]); }

// ================= 출력 제어 =================
void fullOffChannel(uint8_t ch) { pwm.setPWM(ch, 0, 4096); }

void stopTest(bool verbose);

void disableOutputs(bool verbose = true) {
  outputEnabled = false;
  digitalWrite(OE_PIN, HIGH);
  if (i2cOk) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) fullOffChannel(i);
  }
  stopTest(false);
  if (verbose) Serial.println(F("OUTPUT OFF"));
}

void enableOutputs() {
  if (!i2cOk) {
    Serial.println(F("ERROR: PCA9685 I2C not OK. Fix wiring, then reset."));
    return;
  }

  if (mode == Mode::Jog) {
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      if (i != jogServo) fullOffChannel(i);
    }
    currentUs[jogServo] = constrain(currentUs[jogServo], HARD_MIN_US, HARD_MAX_US);
    targetUs[jogServo] = currentUs[jogServo];
    pwm.writeMicroseconds(jogServo, currentUs[jogServo]);
  } else {
    for (uint8_t f = 0; f < NUM_FINGERS; f++) { flexCmd[f] = 0; yawCmd[f] = 0; }
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      currentUs[i] = constrain(servoCal[i].neutralUs, servoCal[i].minUs, servoCal[i].maxUs);
      targetUs[i] = currentUs[i];
      pwm.writeMicroseconds(i, currentUs[i]);
    }
  }

  outputEnabled = true;
  digitalWrite(OE_PIN, LOW);
  lastPacketMs = millis();
  streamFrozen = false;

  Serial.print(F("OUTPUT ON ("));
  if (mode == Mode::Jog)          { Serial.print(F("jog ")); printServoName(jogServo); }
  else if (mode == Mode::Stream)  { Serial.print(F("stream, all neutral")); }
  else                            { Serial.print(F("finger, all neutral")); }
  Serial.println(F(")"));
}

// ================= 논블로킹 모션 엔진 =================
void setServoTarget(uint8_t idx, int us) {
  targetUs[idx] = constrain(us, servoCal[idx].minUs, servoCal[idx].maxUs);
  if (!outputEnabled) currentUs[idx] = targetUs[idx];
}

void setJogTarget(uint8_t idx, int us) {
  targetUs[idx] = constrain(us, HARD_MIN_US, HARD_MAX_US);
  if (!outputEnabled) currentUs[idx] = targetUs[idx];
}

bool motionSettled() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (currentUs[i] != targetUs[i]) return false;
  }
  return true;
}

void serviceMotion() {
  if (!outputEnabled) return;
  const unsigned long now = millis();
  if (now - lastMotionMs < MOTION_TICK_MS) return;
  lastMotionMs = now;

  int step = (int)((long)speedUsPerSec * MOTION_TICK_MS / 1000L);
  if (step < 1) step = 1;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (mode == Mode::Jog && i != jogServo) continue;
    if (currentUs[i] == targetUs[i]) continue;
    if (currentUs[i] < targetUs[i]) currentUs[i] = min(currentUs[i] + step, targetUs[i]);
    else                            currentUs[i] = max(currentUs[i] - step, targetUs[i]);
    pwm.writeMicroseconds(i, currentUs[i]);
  }
}

// ================= flex / yaw 혼합 =================
int calcServoUs(uint8_t idx, int flex, int yaw) {
  const ServoCal &c = servoCal[idx];
  const long fo = (long)c.flexSign * flex * flexSpanUs / CMD_FULL;
  const long yo = (long)c.yawSign  * yaw  * yawSpanUs  / CMD_FULL;
  return constrain((int)((long)c.neutralUs + fo + yo), c.minUs, c.maxUs);
}

void applyFinger(uint8_t f) {
  flexCmd[f] = constrain(flexCmd[f], -CMD_FULL, CMD_FULL);
  yawCmd[f]  = constrain(yawCmd[f],  -CMD_FULL, CMD_FULL);
  const uint8_t o = f * SERVOS_PER_FINGER;
  setServoTarget(o,     calcServoUs(o,     flexCmd[f], yawCmd[f]));
  setServoTarget(o + 1, calcServoUs(o + 1, flexCmd[f], yawCmd[f]));
}

void applyAllFingers() {
  for (uint8_t f = 0; f < NUM_FINGERS; f++) applyFinger(f);
}

void printHandState() {
  for (uint8_t f = 0; f < NUM_FINGERS; f++) {
    Serial.print(FINGER_NAME[f]);
    Serial.print(F(" f="));
    Serial.print(flexCmd[f] / 10);      // 사람이 읽는 단위는 -100..100
    Serial.print(F(" y="));
    Serial.print(yawCmd[f] / 10);
    Serial.print(F(" | "));
    for (uint8_t k = 0; k < SERVOS_PER_FINGER; k++) {
      const uint8_t i = f * SERVOS_PER_FINGER + k;
      printServoName(i);
      Serial.print(F("="));
      Serial.print(currentUs[i]);
      Serial.print(F("us "));
    }
    Serial.println();
  }
  Serial.print(F("output="));
  Serial.println(outputEnabled ? F("ON") : F("OFF"));
}

// ================= 여유(room) 점검 =================
// flex 는 한 방향(0..+CMD_FULL)만 쓴다(ROS/URDF 하한 0). 따라서
//   굽힘 방향(flexSign 쪽)  여유 >= flexSpan + yawSpan
//   반대 방향              여유 >= yawSpan
// 를 만족하면 clamp 없음. 넘으면 명령이 조용히 잘려 GUI 각도와 실제 손이 어긋난다.
void printRoom() {
  Serial.println(F("--- headroom (neutral 기준, flex 는 0..+ 한 방향) ---"));
  bool anyClamp = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    const ServoCal &c = servoCal[i];
    const int up = c.maxUs - c.neutralUs;
    const int dn = c.neutralUs - c.minUs;
    const int flexSide  = (c.flexSign < 0) ? dn : up;   // +flex 가 향하는 쪽
    const int otherSide = (c.flexSign < 0) ? up : dn;
    const bool clamp = (flexSpanUs + yawSpanUs > flexSide) || (yawSpanUs > otherSide);
    if (clamp) anyClamp = true;
    Serial.print(F("  "));
    printServoName(i);
    Serial.print(F("  up=")); Serial.print(up);
    Serial.print(F(" down=")); Serial.print(dn);
    Serial.print(F(" flexSide=")); Serial.print(flexSide);
    Serial.print(F(" need=")); Serial.print(flexSpanUs + yawSpanUs);
    if (clamp) Serial.print(F("  << CLAMP"));
    Serial.println();
  }
  Serial.print(F("  fspan=")); Serial.print(flexSpanUs);
  Serial.print(F(" yspan=")); Serial.print(yawSpanUs);
  Serial.println(anyClamp ? F("  WARNING: 일부 채널에서 명령이 잘린다. span 을 줄이거나 중립을 옮겨라.")
                          : F("  OK (clamp 없음)"));
}

// ================= 동작 테스트 엔진 =================
void stopTest(bool verbose) {
  if (activeTest == Test::None) return;
  activeTest = Test::None;
  activeSeq.poses = nullptr;
  if (verbose) Serial.println(F("TEST STOPPED"));
}

void startSeq(const Pose* table, uint8_t count, const __FlashStringHelper* name) {
  if (!outputEnabled) { Serial.println(F("turn output ON first (type: on)")); return; }
  if (mode != Mode::Finger) { Serial.println(F("finger mode only (type: fm)")); return; }
  activeSeq.poses = table;
  activeSeq.count = count;
  activeTest = Test::Seq;
  testStep = 0;
  testRepeatLeft = testRepeat;
  testWaitingHold = false;
  seqHoldStartMs = 0;
  Serial.print(F("TEST START: "));
  Serial.print(name);
  Serial.print(F(" x"));
  Serial.println(testRepeat);
}

void loadPose(uint8_t idx) {
  Pose p;
  memcpy_P(&p, &activeSeq.poses[idx], sizeof(Pose));
  for (uint8_t f = 0; f < NUM_FINGERS; f++) {
    flexCmd[f] = (int)p.flex[f] * 10;
    yawCmd[f]  = (int)p.yaw[f] * 10;
  }
  applyAllFingers();
  seqHoldMs = p.holdMs;
  seqHoldStartMs = 0;
}

void serviceSeq() {
  if (!testWaitingHold) {
    loadPose(testStep);
    testWaitingHold = true;
    return;
  }
  if (!motionSettled()) return;
  if (seqHoldMs > 0) {
    if (seqHoldStartMs == 0) seqHoldStartMs = millis();
    if (millis() - seqHoldStartMs < seqHoldMs) return;
  }
  seqHoldStartMs = 0;
  testWaitingHold = false;
  testStep++;
  if (testStep >= activeSeq.count) {
    testStep = 0;
    if (testRepeatLeft > 0) testRepeatLeft--;
    if (testRepeatLeft == 0) {
      Serial.println(F("TEST DONE"));
      stopTest(false);
    }
  }
}

constexpr uint16_t SWEEP_DWELL_MS = 500;

void serviceSweep() {
  if (!motionSettled()) return;
  if (seqHoldStartMs == 0) { seqHoldStartMs = millis(); return; }
  if (millis() - seqHoldStartMs < SWEEP_DWELL_MS) return;
  seqHoldStartMs = 0;
  const uint8_t i = testStep;
  const ServoCal &c = servoCal[i];
  switch (testSubStep) {
    case 0: setServoTarget(i, c.minUs);     break;
    case 1: setServoTarget(i, c.neutralUs); break;
    case 2: setServoTarget(i, c.maxUs);     break;
    case 3: setServoTarget(i, c.neutralUs); break;
  }
  if (testSubStep == 0) {
    Serial.print(F("sweep "));
    printServoName(i);
    Serial.print(F(" ["));
    Serial.print(c.minUs);
    Serial.print(F(".."));
    Serial.print(c.maxUs);
    Serial.println(F("]"));
  }
  testSubStep++;
  if (testSubStep > 3) {
    testSubStep = 0;
    testStep++;
    if (testStep >= NUM_SERVOS) {
      Serial.println(F("TEST DONE"));
      stopTest(false);
    }
  }
}

void serviceWave() {
  const unsigned long now = millis();
  if (now - procTimerMs < 40) return;
  procTimerMs = now;
  for (uint8_t f = 0; f < NUM_FINGERS; f++) {
    const float phase = (now / 1200.0f * TWO_PI) - (f * PI / 2.0f);
    flexCmd[f] = (int)(400.0f + 400.0f * sin(phase));   // 0..800 (= 0..80%)
    yawCmd[f] = 0;
  }
  applyAllFingers();
}

void serviceSine() {
  const unsigned long now = millis();
  if (now - procTimerMs < 40) return;
  procTimerMs = now;
  const float pf = now / 2000.0f * TWO_PI;
  const float py = now / 5000.0f * TWO_PI;
  for (uint8_t f = 0; f < NUM_FINGERS; f++) {
    flexCmd[f] = (int)(400.0f + 400.0f * sin(pf));
    yawCmd[f]  = (int)(300.0f * sin(py));
  }
  applyAllFingers();
}

void serviceTest() {
  if (activeTest == Test::None || !outputEnabled) return;
  switch (activeTest) {
    case Test::Seq:   serviceSeq();   break;
    case Test::Sweep: serviceSweep(); break;
    case Test::Wave:  serviceWave();  break;
    case Test::Sine:  serviceSine();  break;
    default: break;
  }
}

// ================= 스트리밍 워치독 =================
void serviceStreamWatchdog() {
  if (mode != Mode::Stream || !outputEnabled) return;
  const unsigned long dt = millis() - lastPacketMs;
  if (!streamFrozen && dt > STREAM_FREEZE_MS) {
    streamFrozen = true;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) targetUs[i] = currentUs[i];
    Serial.println(F("!! STREAM TIMEOUT -> HOLD"));
  }
  if (dt > STREAM_OFF_MS) {
    Serial.println(F("!! STREAM LOST -> OUTPUT OFF"));
    disableOutputs(false);
  }
}

// ================= EEPROM =================
void dumpCal() {
  Serial.println(F("--- calibration ---"));
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    const ServoCal &c = servoCal[i];
    Serial.print(F("  ["));
    Serial.print(i);
    Serial.print(F("] "));
    printServoName(i);
    Serial.print(F("  min="));  Serial.print(c.minUs);
    Serial.print(F(" max="));   Serial.print(c.maxUs);
    Serial.print(F(" neu="));   Serial.print(c.neutralUs);
    Serial.print(F(" flex="));  Serial.print(c.flexSign > 0 ? F("+1") : F("-1"));
    Serial.print(F(" yaw="));   Serial.println(c.yawSign > 0 ? F("+1") : F("-1"));
  }
  Serial.print(F("  flexSpan=")); Serial.print(flexSpanUs);
  Serial.print(F("us yawSpan=")); Serial.print(yawSpanUs);
  Serial.print(F("us speed="));   Serial.print(speedUsPerSec);
  Serial.println(F("us/s"));
}

void saveCal() {
  CalStore s;
  s.magic = CAL_MAGIC;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) s.cal[i] = servoCal[i];
  s.flexSpanUs = (int16_t)flexSpanUs;
  s.yawSpanUs = (int16_t)yawSpanUs;
  s.speedUsPerSec = (int16_t)speedUsPerSec;
  EEPROM.put(0, s);
  Serial.println(F("calibration saved to EEPROM"));
}

bool loadCal(bool verbose) {
  CalStore s;
  EEPROM.get(0, s);
  if (s.magic != CAL_MAGIC) {
    if (verbose) Serial.println(F("no calibration in EEPROM (rev.3+ layout)"));
    return false;
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    s.cal[i].minUs     = constrain(s.cal[i].minUs, HARD_MIN_US, HARD_MAX_US);
    s.cal[i].maxUs     = constrain(s.cal[i].maxUs, HARD_MIN_US, HARD_MAX_US);
    s.cal[i].neutralUs = constrain(s.cal[i].neutralUs, HARD_MIN_US, HARD_MAX_US);
    if (s.cal[i].minUs > s.cal[i].maxUs) {
      const int16_t t = s.cal[i].minUs; s.cal[i].minUs = s.cal[i].maxUs; s.cal[i].maxUs = t;
    }
    s.cal[i].flexSign = (s.cal[i].flexSign >= 0) ? +1 : -1;
    s.cal[i].yawSign  = (s.cal[i].yawSign  >= 0) ? +1 : -1;
    servoCal[i] = s.cal[i];
  }
  flexSpanUs    = constrain(s.flexSpanUs, 0, SPAN_MAX_US);
  yawSpanUs     = constrain(s.yawSpanUs, 0, SPAN_MAX_US);
  speedUsPerSec = constrain(s.speedUsPerSec, SPEED_MIN, SPEED_MAX);
  if (verbose) { Serial.println(F("calibration loaded from EEPROM")); dumpCal(); }
  return true;
}

void wipeCal() {
  EEPROM.put(0, (uint32_t)0);
  Serial.println(F("EEPROM calibration wiped (RAM values unchanged)"));
}

// ================= 도움말 =================
void printHelp() {
  Serial.println(F("=== DexHand 4-finger / 8-servo tool (rev.4) ==="));
  Serial.println(F("CH0 FON CH1 FII | CH2 MON CH3 MIN | CH4 ROI CH5 RII | CH6 LON CH7 LII"));
  Serial.println(F("finger index: 0=Fore 1=Middle 2=Ring 3=Little"));
  Serial.println(F("[output]"));
  Serial.println(F("  on              enable outputs"));
  Serial.println(F("  off | x | enter DISABLE outputs (emergency, works anytime)"));
  Serial.println(F("  sp N            speed 50..3000 us/s"));
  Serial.println(F("[mode] (output OFF only)"));
  Serial.println(F("  j N             jog mode, select servo 0..7"));
  Serial.println(F("  fm              finger mode (flex/yaw)"));
  Serial.println(F("  sm              stream mode (PC / ROS teleop)"));
  Serial.println(F("[jog: per-servo calibration]"));
  Serial.println(F("  a / d           -10 / +10 us      [ / ]   -1 / +1 us"));
  Serial.println(F("  set N           go to N us        c       go to neutral"));
  Serial.println(F("  neu|min|max     record current pulse"));
  Serial.println(F("  fs | ys         flip flexSign | yawSign of selected servo"));
  Serial.println(F("[finger]  (v = -100..100)"));
  Serial.println(F("  f <fi> <v>      flex              y <fi> <v>   yaw"));
  Serial.println(F("  fa <v> | ya <v> all fingers        z            all neutral"));
  Serial.println(F("  ff <fi> | yf <fi>  flip both signs of that finger"));
  Serial.println(F("  ab <servo> <us> direct pulse (clamped)"));
  Serial.println(F("  fspan N | yspan N   span in us (0..700)"));
  Serial.println(F("  room            headroom check (clamp 여부)"));
  Serial.println(F("[motion tests] (finger mode, output ON)"));
  Serial.println(F("  rep N   repeat count for t2..t6 (default 2)"));
  Serial.println(F("  t1  per-servo sweep min/max     t2  finger-by-finger flex"));
  Serial.println(F("  t3  open/close all              t4  wave (phase shifted)"));
  Serial.println(F("  t5  spread (yaw only)           t6  count 1..4"));
  Serial.println(F("  t7  continuous sine (endurance) ts  stop test"));
  Serial.println(F("[stream]"));
  Serial.println(F("  v f0 y0 f1 y1 f2 y2 f3 y3   (-100..100)   legacy"));
  Serial.println(F("  w f0 y0 f1 y1 f2 y2 f3 y3   (-1000..1000) high-res, ROS driver"));
  Serial.println(F("[misc] show | dump | room | save | load | wipe | ping | help"));
}

void printState() {
  Serial.print(F("mode="));
  if (mode == Mode::Jog)         { Serial.print(F("JOG(")); printServoName(jogServo); Serial.print(F(")")); }
  else if (mode == Mode::Stream) { Serial.print(F("STREAM")); }
  else                           { Serial.print(F("FINGER")); }
  Serial.print(F(" output="));
  Serial.print(outputEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" test="));
  Serial.println(activeTest == Test::None ? F("none") : F("running"));
  printHandState();
  dumpCal();
}

// ================= setup =================
void setup() {
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, HIGH);   // 무엇보다 먼저 출력 차단

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(PCA9685_ADDR);
  i2cOk = (Wire.endTransmission() == 0);

  if (i2cOk) {
    pwm.begin();
    pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);
  } else {
    Serial.println(F("ERROR: PCA9685(0x40) not responding."));
    Serial.println(F("Check SDA(20), SCL(21), VCC, GND, then reset."));
  }

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    currentUs[i] = servoCal[i].neutralUs;
    targetUs[i] = currentUs[i];
  }
  for (uint8_t f = 0; f < NUM_FINGERS; f++) { flexCmd[f] = 0; yawCmd[f] = 0; }

  disableOutputs(false);
  Serial.println(F("BOOT: outputs disabled"));

  if (loadCal(false)) {
    Serial.println(F("calibration loaded from EEPROM:"));
    dumpCal();
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      currentUs[i] = servoCal[i].neutralUs;
      targetUs[i] = currentUs[i];
    }
  } else {
    Serial.println(F("using built-in calibration (2026-08-11 measured):"));
    dumpCal();
  }
  printRoom();
  printHelp();
}

// ================= 명령 파서 =================
bool requireOutputOff() {
  if (outputEnabled) { Serial.println(F("turn output OFF first (type: off)")); return false; }
  return true;
}

bool requireMode(Mode m, const __FlashStringHelper* msg) {
  if (mode != m) { Serial.println(msg); return false; }
  return true;
}

bool eq(const char* a, const char* b) { return strcmp(a, b) == 0; }

void handleStreamPacket(char* cmd, int scale) {
  if (mode != Mode::Stream) { Serial.println(F("stream mode only (sm)")); return; }
  int val[NUM_FINGERS * 2];
  const int got = sscanf(cmd + 1, "%d %d %d %d %d %d %d %d",
                         &val[0], &val[1], &val[2], &val[3],
                         &val[4], &val[5], &val[6], &val[7]);
  if (got != NUM_FINGERS * 2) { Serial.println(F("bad packet")); return; }
  for (uint8_t f = 0; f < NUM_FINGERS; f++) {
    flexCmd[f] = val[f * 2] * scale;
    yawCmd[f]  = val[f * 2 + 1] * scale;
  }
  applyAllFingers();
  lastPacketMs = millis();
  streamFrozen = false;
  // 스트리밍 중에는 에코를 찍지 않는다 (대역폭 절약)
}

void handleCommand(char* cmd) {
  if (cmd[0] == '\0' || eq(cmd, "off") || eq(cmd, "x")) { disableOutputs(); return; }

  char verb[12] = {0};
  int  a1 = 0, a2 = 0;
  const int nargs = sscanf(cmd, "%11s %d %d", verb, &a1, &a2);

  // ---- 스트리밍 패킷 (가장 자주 오므로 먼저 본다) ----
  if (eq(verb, "w")) { handleStreamPacket(cmd, 1);  return; }
  if (eq(verb, "v")) { handleStreamPacket(cmd, 10); return; }

  // ---- 출력, 공통 ----
  if (eq(verb, "on"))        { enableOutputs(); return; }
  if (eq(verb, "help") || eq(verb, "?")) { printHelp(); return; }
  if (eq(verb, "show"))      { printState(); return; }
  if (eq(verb, "dump"))      { dumpCal(); return; }
  if (eq(verb, "room"))      { printRoom(); return; }
  if (eq(verb, "save"))      { saveCal(); return; }
  if (eq(verb, "load"))      { loadCal(true); return; }
  if (eq(verb, "wipe"))      { wipeCal(); return; }
  if (eq(verb, "ts"))        { stopTest(true); return; }
  if (eq(verb, "ping"))      { Serial.println(F("pong")); return; }
  if (eq(verb, "sp") && nargs >= 2) {
    speedUsPerSec = constrain(a1, SPEED_MIN, SPEED_MAX);
    Serial.print(F("speed=")); Serial.print(speedUsPerSec); Serial.println(F("us/s"));
    return;
  }
  if (eq(verb, "rep") && nargs >= 2) {
    defaultRepeats = constrain(a1, 1, 100);
    testRepeat = (uint8_t)defaultRepeats;
    Serial.print(F("repeat=")); Serial.println(testRepeat);
    return;
  }

  // ---- 모드 전환 ----
  if (eq(verb, "j") && nargs >= 2) {
    if (!requireOutputOff()) return;
    if (a1 < 0 || a1 >= NUM_SERVOS) { Serial.println(F("usage: j 0..7")); return; }
    mode = Mode::Jog; jogServo = (uint8_t)a1;
    Serial.print(F("JOG mode, servo=")); printServoName(jogServo);
    Serial.print(F(" (CH")); Serial.print(jogServo); Serial.println(F(")"));
    return;
  }
  if (eq(verb, "fm")) {
    if (!requireOutputOff()) return;
    mode = Mode::Finger;
    for (uint8_t f = 0; f < NUM_FINGERS; f++) { flexCmd[f] = 0; yawCmd[f] = 0; }
    Serial.println(F("FINGER mode. Check neutrals below:"));
    dumpCal();
    return;
  }
  if (eq(verb, "sm")) {
    if (!requireOutputOff()) return;
    mode = Mode::Stream;
    for (uint8_t f = 0; f < NUM_FINGERS; f++) { flexCmd[f] = 0; yawCmd[f] = 0; }
    Serial.println(F("STREAM mode. Send: w f0 y0 f1 y1 f2 y2 f3 y3  (-1000..1000)"));
    return;
  }

  // ---- jog ----
  if (mode == Mode::Jog) {
    bool handled = true;
    if      (eq(verb, "a"))  setJogTarget(jogServo, targetUs[jogServo] - 10);
    else if (eq(verb, "d"))  setJogTarget(jogServo, targetUs[jogServo] + 10);
    else if (eq(verb, "["))  setJogTarget(jogServo, targetUs[jogServo] - 1);
    else if (eq(verb, "]"))  setJogTarget(jogServo, targetUs[jogServo] + 1);
    else if (eq(verb, "c"))  setJogTarget(jogServo, servoCal[jogServo].neutralUs);
    else if (eq(verb, "set") && nargs >= 2) setJogTarget(jogServo, a1);
    else if (eq(verb, "neu") || eq(verb, "min") || eq(verb, "max")) {
      ServoCal &c = servoCal[jogServo];
      const int v = targetUs[jogServo];
      if      (eq(verb, "neu")) c.neutralUs = v;
      else if (eq(verb, "min")) c.minUs = v;
      else                      c.maxUs = v;
      if (c.minUs > c.maxUs) {
        const int16_t t = c.minUs; c.minUs = c.maxUs; c.maxUs = t;
        Serial.println(F("(min > max 였으므로 두 값을 서로 바꿔 저장)"));
      }
      printServoName(jogServo);
      Serial.print(F(" ")); Serial.print(verb);
      Serial.print(F(" = ")); Serial.print(v);
      Serial.println(F("us 기록됨. save 로 EEPROM 저장."));
      return;
    }
    else if (eq(verb, "fs")) {
      servoCal[jogServo].flexSign = -servoCal[jogServo].flexSign;
      Serial.print(F("flexSign ")); printServoName(jogServo);
      Serial.println(servoCal[jogServo].flexSign > 0 ? F(" = +1") : F(" = -1"));
      return;
    }
    else if (eq(verb, "ys")) {
      servoCal[jogServo].yawSign = -servoCal[jogServo].yawSign;
      Serial.print(F("yawSign ")); printServoName(jogServo);
      Serial.println(servoCal[jogServo].yawSign > 0 ? F(" = +1") : F(" = -1"));
      return;
    }
    else handled = false;

    if (handled) {
      printServoName(jogServo);
      Serial.print(F(" target=")); Serial.print(targetUs[jogServo]);
      Serial.println(outputEnabled ? F("us [ON]") : F("us [OFF, target only]"));
      return;
    }
  }

  // ---- 스팬 (모드 무관) ----
  if (eq(verb, "fspan") && nargs >= 2) {
    flexSpanUs = constrain(a1, 0, SPAN_MAX_US);
    Serial.print(F("flexSpanUs=")); Serial.println(flexSpanUs);
    printRoom();
    if (mode != Mode::Jog) applyAllFingers();
    return;
  }
  if (eq(verb, "yspan") && nargs >= 2) {
    yawSpanUs = constrain(a1, 0, SPAN_MAX_US);
    Serial.print(F("yawSpanUs=")); Serial.println(yawSpanUs);
    printRoom();
    if (mode != Mode::Jog) applyAllFingers();
    return;
  }

  // ---- finger ----
  if (eq(verb, "f") && nargs >= 3) {
    if (!requireMode(Mode::Finger, F("finger mode only (fm)"))) return;
    if (a1 < 0 || a1 >= NUM_FINGERS) { Serial.println(F("finger 0..3")); return; }
    stopTest(false);
    flexCmd[a1] = a2 * 10; applyFinger(a1); printHandState();
    return;
  }
  if (eq(verb, "y") && nargs >= 3) {
    if (!requireMode(Mode::Finger, F("finger mode only (fm)"))) return;
    if (a1 < 0 || a1 >= NUM_FINGERS) { Serial.println(F("finger 0..3")); return; }
    stopTest(false);
    yawCmd[a1] = a2 * 10; applyFinger(a1); printHandState();
    return;
  }
  if (eq(verb, "fa") && nargs >= 2) {
    if (!requireMode(Mode::Finger, F("finger mode only (fm)"))) return;
    stopTest(false);
    for (uint8_t f = 0; f < NUM_FINGERS; f++) flexCmd[f] = a1 * 10;
    applyAllFingers(); printHandState();
    return;
  }
  if (eq(verb, "ya") && nargs >= 2) {
    if (!requireMode(Mode::Finger, F("finger mode only (fm)"))) return;
    stopTest(false);
    for (uint8_t f = 0; f < NUM_FINGERS; f++) yawCmd[f] = a1 * 10;
    applyAllFingers(); printHandState();
    return;
  }
  if (eq(verb, "z")) {
    stopTest(false);
    for (uint8_t f = 0; f < NUM_FINGERS; f++) { flexCmd[f] = 0; yawCmd[f] = 0; }
    applyAllFingers(); printHandState();
    return;
  }
  if (eq(verb, "ff") && nargs >= 2) {
    if (a1 < 0 || a1 >= NUM_FINGERS) { Serial.println(F("finger 0..3")); return; }
    for (uint8_t k = 0; k < SERVOS_PER_FINGER; k++) {
      ServoCal &c = servoCal[a1 * SERVOS_PER_FINGER + k];
      c.flexSign = -c.flexSign;
    }
    dumpCal();
    return;
  }
  if (eq(verb, "yf") && nargs >= 2) {
    if (a1 < 0 || a1 >= NUM_FINGERS) { Serial.println(F("finger 0..3")); return; }
    for (uint8_t k = 0; k < SERVOS_PER_FINGER; k++) {
      ServoCal &c = servoCal[a1 * SERVOS_PER_FINGER + k];
      c.yawSign = -c.yawSign;
    }
    dumpCal();
    return;
  }
  if (eq(verb, "ab") && nargs >= 3) {
    if (a1 < 0 || a1 >= NUM_SERVOS) { Serial.println(F("servo 0..7")); return; }
    stopTest(false);
    setServoTarget(a1, a2);
    printServoName(a1);
    Serial.print(F(" target=")); Serial.print(targetUs[a1]); Serial.println(F("us"));
    return;
  }

  // ---- 테스트 ----
  if (verb[0] == 't' && verb[1] >= '1' && verb[1] <= '7' && verb[2] == '\0') {
    testRepeat = (uint8_t)defaultRepeats;
    switch (verb[1]) {
      case '1':
        if (!outputEnabled) { Serial.println(F("turn output ON first")); return; }
        if (mode != Mode::Finger) { Serial.println(F("finger mode only (fm)")); return; }
        activeTest = Test::Sweep; testStep = 0; testSubStep = 0; seqHoldStartMs = 0;
        Serial.println(F("TEST START: per-servo sweep"));
        Serial.println(F("WARNING: uses each servo's recorded min/max. Calibrate first!"));
        break;
      case '2':
        startSeq(POSE_SEQUENTIAL, sizeof(POSE_SEQUENTIAL) / sizeof(Pose), F("finger-by-finger"));
        break;
      case '3':
        startSeq(POSE_OPENCLOSE, sizeof(POSE_OPENCLOSE) / sizeof(Pose), F("open/close"));
        break;
      case '4':
        if (!outputEnabled || mode != Mode::Finger) { Serial.println(F("need finger mode + output ON")); return; }
        activeTest = Test::Wave; procTimerMs = millis();
        Serial.println(F("TEST START: wave (ts to stop)"));
        break;
      case '5':
        startSeq(POSE_SPREAD, sizeof(POSE_SPREAD) / sizeof(Pose), F("spread"));
        break;
      case '6':
        startSeq(POSE_COUNT, sizeof(POSE_COUNT) / sizeof(Pose), F("count 1..4"));
        break;
      case '7':
        if (!outputEnabled || mode != Mode::Finger) { Serial.println(F("need finger mode + output ON")); return; }
        activeTest = Test::Sine; procTimerMs = millis();
        Serial.println(F("TEST START: continuous sine (ts to stop)"));
        break;
    }
    return;
  }

  printHelp();
}

// ================= loop =================
void loop() {
  serviceTest();
  serviceMotion();
  serviceStreamWatchdog();

  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      cmdBuf[cmdLen] = '\0';
      for (uint8_t i = 0; i < cmdLen; i++) cmdBuf[i] = (char)tolower(cmdBuf[i]);
      handleCommand(cmdBuf);
      cmdLen = 0;
      continue;
    }
    if (cmdLen < CMD_BUF_SIZE - 1) cmdBuf[cmdLen++] = ch;
  }
}
