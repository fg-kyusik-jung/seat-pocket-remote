// 파일럿 3: 능동형/실시간/매뉴얼 통합 UI + 하드웨어 제어
// 4포켓/2모터 통합 하드웨어. 밸브(6채널)/LED(3)/스위치(3)는 PCF8574 2개로 확장,
// LCD는 PCF8574 인터페이스 내장형 I2C 1602. 전부 같은 I2C 버스(A4/A5)에 병렬 연결.
// UI(모드선택/캘리브레이션 연출/자세보정/포켓조절/프리셋)는 io_switch_test.ino에서 하드웨어 없이 먼저 검증 완료.
// 이 파일은 그 UI 뒤에 실제 밸브/펌프/센서 제어를 연결한 최종본 — 2way 밸브 도착 후 바로 검증 가능.
//
// ⚠ 아직 실측 안 된 가정 (2way 밸브 배선 후 반드시 검증):
//   - VALVE_ACTIVE_HIGH: 밸브 릴레이 익스팬더 출력 극성 (에너자이즈 시 HIGH인지 LOW인지)
//   - PUMP_FIXED_DIR: 파일럿1 실측치를 그대로 가정 (모터1/2 둘 다 동일 극성 배선이라는 가정, 개별 확인 필요)
//   - ADJUST_HOLD_TIMEOUT_MS(매뉴얼 조절 홀드 판정 시간), 프리셋 적용 시 목표수렴 속도 — 실제 배관 연결 후 튜닝 필요
// ✅ 이미 실측/확정된 것: LED_ACTIVE_LOW=false, IO익스팬더 0x24, 밸브익스팬더 0x20, LCD 핀배치(RS/RW/EN/BL/D4~D7), 엔코더 배선

#include <Wire.h>
#include <EEPROM.h>

// ---------- I2C 주소 ----------
const uint8_t ADDR_LCD = 0x27;
const uint8_t ADDR_IO = 0x24;    // LED 3 + 스위치 3 (2026-07-23: 실측상 두 모듈 주소가 반대였음, 스왑함)
const uint8_t ADDR_VALVE = 0x20; // 밸브 릴레이 6채널 (2026-07-23: 스왑됨)

// ---------- 아두이노 직결 핀 ----------
const uint8_t ENC_CLK_PIN = 2;  // 인터럽트
const uint8_t ENC_DT_PIN = 3;
const uint8_t ENC_SW_PIN = 4;

const uint8_t MOTOR2_IN1_PIN = 5; // 모터2(그룹2, 포켓2/3) IN1, Timer0
const uint8_t MOTOR2_IN2_PIN = 6; // 모터2 IN2, Timer0
const uint8_t BUZZER_PIN = 7;
const uint8_t MOTOR1_IN1_PIN = 9;  // 모터1(그룹1, 포켓0/1) IN1, Timer1
const uint8_t MOTOR1_IN2_PIN = 10; // 모터1 IN2, Timer1

const uint8_t SENSOR_PINS[4] = {A0, A1, A2, A3};

// ---------- 그룹/포켓 매핑 ----------
// 그룹1 = 포켓0,1 (모터1) / 그룹2 = 포켓2,3 (모터2)
const uint8_t POCKET_GROUP[4] = {0, 0, 1, 1};

// ---------- 밸브 익스팬더(ADDR_VALVE) 비트 배치 ----------
const uint8_t BIT_3WAY_G1 = 0;
const uint8_t BIT_2WAY_P0 = 1;
const uint8_t BIT_2WAY_P1 = 2;
const uint8_t BIT_3WAY_G2 = 3;
const uint8_t BIT_2WAY_P2 = 4;
const uint8_t BIT_2WAY_P3 = 5;
const uint8_t BIT_2WAY_POCKET[4] = {BIT_2WAY_P0, BIT_2WAY_P1, BIT_2WAY_P2, BIT_2WAY_P3};
const uint8_t BIT_3WAY_GROUP[2] = {BIT_3WAY_G1, BIT_3WAY_G2};

// ---------- IO 익스팬더(ADDR_IO) 비트 배치 ----------
const uint8_t BIT_LED[3] = {0, 1, 2};   // 프리셋 1/2/3 인디케이터
const uint8_t BIT_SW[3] = {3, 4, 5};    // 프리셋 1/2/3 불러오기 버튼

// ---------- 극성 가정 ----------
const bool VALVE_ACTIVE_HIGH = true; // true면 1=에너자이즈(2way 열림/3way 펌프모드) — 실측 필요
const bool LED_ACTIVE_LOW = false;   // 2026-07-21 실측 확정
const uint8_t PUMP_FIXED_DIR = 0;    // 파일럿1 실측치 가정 — 개별 재확인 필요

// ---------- 튜닝 파라미터 (시리얼로 조정 가능) ----------
uint16_t pressureCeilingRaw = 500;
uint16_t pressureFloorRaw = 120;
uint16_t toleranceRaw = 5;
uint8_t pumpSpeed = 255;
const unsigned long SWEEP_TICK_MS = 100;
const unsigned long INFLATE_TIMEOUT_MS = 30000;
const unsigned long SWEEP_TIMEOUT_MS = 60000;

const unsigned long LONG_PRESS_MS = 1000;
const unsigned long SW_DEBOUNCE_MS = 150;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;

// ---------- 시스템 전원 / 엔코더 스위치(클릭) ----------
bool systemOn = false;
bool encSwPrevState = HIGH;
unsigned long encSwPressStartMs = 0;
unsigned long encSwLastChangeMs = 0;
bool longPressFired = false;

bool clickPending = false;
unsigned long clickPendingMs = 0;
const unsigned long DOUBLE_CLICK_WINDOW_MS = 400;

int8_t currentModeIdx = 0; // 0=AUTO FIT, 1=REALTIME, 2=MANUAL

// 프리셋 전역 단축키 상태 (핸들러/함수는 아래쪽에 정의, 변수만 먼저 선언)
bool presetApplying = false;
uint8_t presetApplyingIdx = 0;
uint16_t presetPreviousRaw[4] = {0, 0, 0, 0}; // 프리셋 적용 시작 직전 압력 (중단 시 이 값으로 복귀)
bool presetReverting = false; // 중단 후 직전 상태로 되돌아가는 중(완료 처리 분기용)
bool presetFrozen = false;    // 프리셋 적용 완료 후 사용자 입력 있을 때까지 화면 고정

// ---------- 엔코더 회전 (풀 쿼드러처 상태기계 디코드) ----------
// 기존엔 CLK 하강엣지 1곳만 보고 그 순간 DT값으로 방향을 판정 + 시간 디바운스로
// 걸렀는데, 새 엔코더 모듈에서 "천천히 한 칸씩 돌릴 때는 정확하지만 보통 속도로
// 돌리면 +1/-1이 번갈아 찍히며 서로 상쇄돼 아예 안 움직이는" 현상이 실측으로
// 확인됨(그레이코드 전이 중 순간적인 바운스/노이즈가 방향 반전처럼 잘못 읽힘).
// CLK+DT 두 핀 전이를 전부 추적하는 상태기계로 바꾸면, 바운스로 인한 왕복 전이는
// +1/-1이 누적기 안에서 서로 상쇄되고 진짜 한 칸(그레이코드 4상태 전부)이 완주됐을
// 때만 스텝 하나가 나가서 훨씬 강건함 - 시간 기반 디바운스 자체가 불필요해짐.
volatile int8_t rotAccum = 0;
volatile uint8_t encRawState = 0;   // 직전 (CLK<<1 | DT) 2비트 상태
volatile int8_t encQuarterAccum = 0; // 한 클릭(그레이코드 4상태)을 채울 때까지의 누적

// idx = (이전상태<<2 | 현재상태), 값은 그 전이가 CW(+1)/CCW(-1)/무효또는무변화(0)인지
const int8_t ENC_QUAD_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0,
};

bool encDebugEnabled = false; // 시리얼 ENCDEBUG,1 로 켜기 - 실제 반영된 스텝(클릭)만 기록
const uint8_t ENC_LOG_SIZE = 32;
volatile unsigned long encLogUs[ENC_LOG_SIZE];
volatile int8_t encLogDir[ENC_LOG_SIZE];
volatile uint8_t encLogHead = 0;
volatile uint8_t encLogCount = 0;

void logEncStep(unsigned long now, int8_t dir) {
  encLogUs[encLogHead] = now;
  encLogDir[encLogHead] = dir;
  encLogHead = (encLogHead + 1) % ENC_LOG_SIZE;
  if (encLogCount < ENC_LOG_SIZE) encLogCount++;
}

void encoderISR() {
  uint8_t clk = digitalRead(ENC_CLK_PIN);
  uint8_t dt = digitalRead(ENC_DT_PIN);
  uint8_t state = (clk << 1) | dt;
  uint8_t idx = (encRawState << 2) | state;
  encRawState = state;
  encQuarterAccum += ENC_QUAD_TABLE[idx & 0x0F];

  if (encQuarterAccum >= 4) {
    rotAccum++;
    encQuarterAccum = 0;
    if (encDebugEnabled) logEncStep(micros(), 1);
  } else if (encQuarterAccum <= -4) {
    rotAccum--;
    encQuarterAccum = 0;
    if (encDebugEnabled) logEncStep(micros(), -1);
  }

  // 정지(디텐트) 위치 = 양쪽 핀 다 HIGH(풀업, 회전 안 하는 중). 여기로 돌아올 때마다
  // 누적값을 강제로 0에 맞춰서, 중간에 노이즈로 살짝 어긋난 잔여값이 다음 클릭까지
  // 이어져 "몇 번은 반응 없다가 몰아서 반영되는" 현상을 방지 (2026-08-05)
  if (state == 3) {
    encQuarterAccum = 0;
  }
}

void tickEncDebug() {
  if (!encDebugEnabled || encLogCount == 0) return;
  noInterrupts();
  uint8_t count = encLogCount;
  uint8_t head = encLogHead;
  encLogCount = 0;
  interrupts();

  static unsigned long prevUs = 0;
  uint8_t start = (head + ENC_LOG_SIZE - count) % ENC_LOG_SIZE;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = (start + i) % ENC_LOG_SIZE;
    unsigned long us = encLogUs[idx];
    long deltaUs = (prevUs == 0) ? 0 : (long)(us - prevUs);
    Serial.print(F("ENC,delta_us="));
    Serial.print(deltaUs);
    Serial.print(F(",dir="));
    Serial.println(encLogDir[idx]);
    prevUs = us;
  }
}

// ---------- IO/밸브 익스팬더 섀도 레지스터 ----------
uint8_t ioShadow = 0xFF;
uint8_t valveShadow = 0x00; // 기본값: 전부 비에너자이즈(안전)

unsigned long lastSwPollMs = 0;
bool swWasPressed[3] = {false, false, false};

String inputLine;
unsigned long lastDebugPrintMs = 0;
bool debugPrintEnabled = true; // 시리얼 DEBUGPRINT,0 으로 끄면 명령 입력하기 편함 (DEBUGPRINT,1로 재개)

// ================= I2C 저수준 =================
void pcfWrite(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t pcfRead(uint8_t addr) {
  Wire.requestFrom((int)addr, 1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// ================= LCD (1602, PCF8574 내장, 4비트 모드) =================
const uint8_t LCD_BIT_RS = 0;
const uint8_t LCD_BIT_EN = 2;
const uint8_t LCD_BIT_BL = 3;

void lcdPulseEnable(uint8_t data) {
  pcfWrite(ADDR_LCD, data | (1 << LCD_BIT_EN));
  delayMicroseconds(1);
  pcfWrite(ADDR_LCD, data & ~(1 << LCD_BIT_EN));
  delayMicroseconds(50);
}

void lcdWrite4(uint8_t nibble, bool rs) {
  uint8_t data = (1 << LCD_BIT_BL);
  if (rs) data |= (1 << LCD_BIT_RS);
  if (nibble & 0x1) data |= (1 << 4);
  if (nibble & 0x2) data |= (1 << 5);
  if (nibble & 0x4) data |= (1 << 6);
  if (nibble & 0x8) data |= (1 << 7);
  pcfWrite(ADDR_LCD, data);
  lcdPulseEnable(data);
}

void lcdCommand(uint8_t cmd) {
  lcdWrite4(cmd >> 4, false);
  lcdWrite4(cmd & 0x0F, false);
  delay(2);
}

void lcdChar(char c) {
  lcdWrite4(c >> 4, true);
  lcdWrite4(c & 0x0F, true);
  delayMicroseconds(50);
}

void lcdInit() {
  delay(50);
  lcdWrite4(0x3, false);
  delay(5);
  lcdWrite4(0x3, false);
  delayMicroseconds(150);
  lcdWrite4(0x3, false);
  lcdWrite4(0x2, false);
  lcdCommand(0x28);
  lcdCommand(0x0C);
  lcdCommand(0x06);
  lcdCommand(0x01);
  delay(2);
}

void lcdSetCursor(uint8_t col, uint8_t row) {
  uint8_t rowOffset = row == 0 ? 0x00 : 0x40;
  lcdCommand(0x80 | (rowOffset + col));
}

void lcdPrint(const char *s) {
  while (*s) lcdChar(*s++);
}

void lcdClear() {
  lcdCommand(0x01);
  delay(2);
}

void lcdPrintLine(uint8_t row, const char *s) {
  lcdSetCursor(0, row);
  char buf[17];
  uint8_t i = 0;
  for (; i < 16 && s[i]; i++) buf[i] = s[i];
  for (; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';
  lcdPrint(buf);
}

void lcdBacklightOff() {
  pcfWrite(ADDR_LCD, 0x00);
}

void lcdBacklightOn() {
  pcfWrite(ADDR_LCD, (1 << LCD_BIT_BL));
}

void lcdCreateChar(uint8_t location, const uint8_t *charmap) {
  lcdCommand(0x40 | ((location & 0x7) << 3));
  for (uint8_t i = 0; i < 8; i++) lcdChar(charmap[i]);
}

const uint8_t CHAR_BOOKMARK = 1;
const uint8_t BOOKMARK_GLYPH[8] = {0x1F, 0x11, 0x11, 0x11, 0x11, 0x15, 0x1B, 0x00};

// ================= IO 익스팬더 (LED + 스위치) =================
void setLed(uint8_t idx, bool on) {
  bool bitHigh = LED_ACTIVE_LOW ? !on : on;
  if (bitHigh) ioShadow |= (1 << BIT_LED[idx]);
  else ioShadow &= ~(1 << BIT_LED[idx]);
  pcfWrite(ADDR_IO, ioShadow);
}

void setLedExclusive(int8_t idx) {
  for (uint8_t i = 0; i < 3; i++) setLed(i, i == idx);
}

bool readSwitch(uint8_t idx) {
  uint8_t writeVal = ioShadow;
  for (uint8_t i = 0; i < 3; i++) writeVal |= (1 << BIT_SW[i]);
  if (writeVal != ioShadow) {
    ioShadow = writeVal;
    pcfWrite(ADDR_IO, ioShadow);
  }
  uint8_t raw = pcfRead(ADDR_IO);
  return (raw & (1 << BIT_SW[idx])) == 0; // LOW = 눌림
}

// ================= 밸브 익스팬더 =================
void writeValveBit(uint8_t bit, bool energize) {
  bool bitHigh = VALVE_ACTIVE_HIGH ? energize : !energize;
  if (bitHigh) valveShadow |= (1 << bit);
  else valveShadow &= ~(1 << bit);
  pcfWrite(ADDR_VALVE, valveShadow);
}

void setValve2way(uint8_t pocketIdx, bool open) {
  writeValveBit(BIT_2WAY_POCKET[pocketIdx], open);
}

void setValve3way(uint8_t groupIdx, bool pumpMode) {
  writeValveBit(BIT_3WAY_GROUP[groupIdx], pumpMode);
}

void closeAllValves() {
  for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
  for (uint8_t g = 0; g < 2; g++) setValve3way(g, false);
}

// ================= 펌프 =================
void stopPump(uint8_t groupIdx) {
  if (groupIdx == 0) {
    analogWrite(MOTOR1_IN1_PIN, 0);
    analogWrite(MOTOR1_IN2_PIN, 0);
  } else {
    analogWrite(MOTOR2_IN1_PIN, 0);
    analogWrite(MOTOR2_IN2_PIN, 0);
  }
}

void runPump(uint8_t groupIdx, uint8_t speed) {
  uint8_t pinA = groupIdx == 0 ? MOTOR1_IN1_PIN : MOTOR2_IN1_PIN;
  uint8_t pinB = groupIdx == 0 ? MOTOR1_IN2_PIN : MOTOR2_IN2_PIN;
  if (PUMP_FIXED_DIR == 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, speed);
  }
}

void stopAllPumps() {
  stopPump(0);
  stopPump(1);
}

void stopEverythingSafely() {
  stopAllPumps();
  closeAllValves();
  noTone(BUZZER_PIN);
}

// ================= 센서 =================
uint16_t sensorRaw[4];

void readAllSensors() {
  for (uint8_t i = 0; i < 4; i++) sensorRaw[i] = analogRead(SENSOR_PINS[i]);
}

uint8_t rawToLevel(uint16_t raw) {
  int32_t clamped = raw;
  if (clamped < 80) clamped = 80;
  if (clamped > 500) clamped = 500;
  int32_t level = ((clamped - 80) * 10) / 420 + 1;
  if (level > 10) level = 10;
  if (level < 1) level = 1;
  return (uint8_t)level;
}

uint16_t levelToRawTarget(uint8_t level) {
  if (level == 0) return 80; // 프리셋 전용 특수값: 완전 배기(대기압 근처)
  if (level > 10) level = 10;
  return 80 + (uint16_t)(level - 1) * 42 + 21; // 해당 레벨 구간의 대략적 중간값
}

// ================= 그룹별 목표압력 추적 (능동형 RETURN 단계 / 프리셋 적용 공용) =================
enum ReturnPhase { RP_INFLATE, RP_DEFLATE, RP_DONE };
ReturnPhase returnPhase[2] = {RP_DONE, RP_DONE};
uint16_t targetRaw[4] = {0, 0, 0, 0};

void startTargetSeek(const uint16_t targets[4]) {
  for (uint8_t i = 0; i < 4; i++) targetRaw[i] = targets[i];
  returnPhase[0] = RP_INFLATE;
  returnPhase[1] = RP_INFLATE;
}

void tickTargetSeekGroup(uint8_t groupIdx) {
  if (returnPhase[groupIdx] == RP_DONE) return;

  readAllSensors();
  uint8_t p0 = groupIdx == 0 ? 0 : 2;
  uint8_t p1 = groupIdx == 0 ? 1 : 3;

  if (returnPhase[groupIdx] == RP_INFLATE) {
    bool need0 = sensorRaw[p0] + toleranceRaw < targetRaw[p0];
    bool need1 = sensorRaw[p1] + toleranceRaw < targetRaw[p1];
    if (!need0 && !need1) {
      stopPump(groupIdx);
      setValve2way(p0, false);
      setValve2way(p1, false);
      returnPhase[groupIdx] = RP_DEFLATE;
      return;
    }
    setValve3way(groupIdx, true);
    setValve2way(p0, need0);
    setValve2way(p1, need1);
    runPump(groupIdx, pumpSpeed);
  } else if (returnPhase[groupIdx] == RP_DEFLATE) {
    bool need0 = sensorRaw[p0] > targetRaw[p0] + toleranceRaw;
    bool need1 = sensorRaw[p1] > targetRaw[p1] + toleranceRaw;
    if (!need0 && !need1) {
      setValve2way(p0, false);
      setValve2way(p1, false);
      setValve3way(groupIdx, false);
      returnPhase[groupIdx] = RP_DONE;
      return;
    }
    setValve3way(groupIdx, false);
    setValve2way(p0, need0);
    setValve2way(p1, need1);
  }
}

bool tickTargetSeek() {
  tickTargetSeekGroup(0);
  tickTargetSeekGroup(1);
  return returnPhase[0] == RP_DONE && returnPhase[1] == RP_DONE;
}

// ================= 능동형 모드 하드웨어 상태머신 (모드1 캘리브레이션의 실제 엔진) =================
enum ActiveState { AS_RESET, AS_INFLATE, AS_SWEEP, AS_RETURN, AS_LOCKED };
ActiveState activeState = AS_LOCKED;
unsigned long activeStateEnteredMs = 0;
unsigned long lastSweepTickMs = 0;

uint16_t resetTargetRaw = 90; // 캘리브레이션 시작 전 초기화(완전배기) 목표치, 시리얼 RESETRAW,<raw> 로 조정
const unsigned long RESET_TIMEOUT_MS = 10000;

bool bestFound = false;
uint16_t bestAvg = 0xFFFF;
uint16_t bestTarget[4] = {0, 0, 0, 0};

bool buzzerActive = false;
void updateBuzzer(bool shouldBeep) {
  if (shouldBeep && !buzzerActive) {
    tone(BUZZER_PIN, 440);
    buzzerActive = true;
  } else if (!shouldBeep && buzzerActive) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
}

// 캘리브레이션 시작 전 초기화 단계: 현재 압력 상태와 무관하게 일단 4포켓 전부 배기해서
// 항상 같은 기준(빈 상태)에서 INFLATE를 시작하도록 함
void enterReset() {
  activeState = AS_RESET;
  activeStateEnteredMs = millis();
  stopAllPumps();
  setValve3way(0, false);
  setValve3way(1, false);
  for (uint8_t i = 0; i < 4; i++) setValve2way(i, true);
}

void tickReset() {
  readAllSensors();
  bool allVented = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (sensorRaw[i] > resetTargetRaw) allVented = false;
  }
  bool timedOut = (millis() - activeStateEnteredMs) > RESET_TIMEOUT_MS;
  if (allVented || timedOut) {
    Serial.print(F("RESET 종료: "));
    Serial.print(millis() - activeStateEnteredMs);
    Serial.println(timedOut ? F("ms (timeout)") : F("ms (배기 완료)"));
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    enterInflate();
  }
}

void enterInflate() {
  activeState = AS_INFLATE;
  activeStateEnteredMs = millis();
  bestFound = false;
  bestAvg = 0xFFFF;
  setValve3way(0, true);
  setValve3way(1, true);
  for (uint8_t i = 0; i < 4; i++) setValve2way(i, true);
}

void tickInflate() {
  readAllSensors();
  bool allDone = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (sensorRaw[i] >= pressureCeilingRaw) {
      setValve2way(i, false);
    } else {
      allDone = false;
    }
  }
  bool group1Running = sensorRaw[0] < pressureCeilingRaw || sensorRaw[1] < pressureCeilingRaw;
  bool group2Running = sensorRaw[2] < pressureCeilingRaw || sensorRaw[3] < pressureCeilingRaw;
  if (group1Running) runPump(0, pumpSpeed); else stopPump(0);
  if (group2Running) runPump(1, pumpSpeed); else stopPump(1);

  updateBuzzer(sensorRaw[0] >= pressureCeilingRaw || sensorRaw[1] >= pressureCeilingRaw ||
               sensorRaw[2] >= pressureCeilingRaw || sensorRaw[3] >= pressureCeilingRaw);

  if (allDone || (millis() - activeStateEnteredMs) > INFLATE_TIMEOUT_MS) {
    Serial.print(F("INFLATE 종료: "));
    Serial.print(millis() - activeStateEnteredMs);
    Serial.println(allDone ? F("ms (ceiling 도달)") : F("ms (timeout)"));
    stopAllPumps();
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    updateBuzzer(false);
    activeState = AS_SWEEP;
    activeStateEnteredMs = millis();
    setValve3way(0, false);
    setValve3way(1, false);
  }
}

void tickSweep() {
  if (millis() - lastSweepTickMs < SWEEP_TICK_MS) return;
  lastSweepTickMs = millis();

  readAllSensors();
  uint16_t refLow = sensorRaw[0];
  for (uint8_t i = 1; i < 4; i++) if (sensorRaw[i] < refLow) refLow = sensorRaw[i];

  bool inTolerance = true;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (sensorRaw[i] > refLow + toleranceRaw) inTolerance = false;
    sum += sensorRaw[i];
  }
  uint16_t avg = sum / 4;

  if (inTolerance && (!bestFound || avg < bestAvg)) {
    bestFound = true;
    bestAvg = avg;
    for (uint8_t i = 0; i < 4; i++) bestTarget[i] = sensorRaw[i];
  }

  for (uint8_t i = 0; i < 4; i++) {
    bool needsBleed = sensorRaw[i] > refLow + toleranceRaw;
    setValve2way(i, needsBleed);
  }

  bool atFloor = refLow <= pressureFloorRaw;
  bool timedOut = (millis() - activeStateEnteredMs) > SWEEP_TIMEOUT_MS;
  if (atFloor || timedOut) {
    Serial.print(F("SWEEP 종료: "));
    Serial.print(millis() - activeStateEnteredMs);
    Serial.println(atFloor ? F("ms (floor 도달)") : F("ms (timeout)"));
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    if (!bestFound) {
      bestFound = true;
      for (uint8_t i = 0; i < 4; i++) bestTarget[i] = sensorRaw[i];
    }
    activeState = AS_RETURN;
    activeStateEnteredMs = millis();
    startTargetSeek(bestTarget);
  }
}

void tickReturn() {
  if (tickTargetSeek()) {
    Serial.print(F("RETURN 종료: "));
    Serial.print(millis() - activeStateEnteredMs);
    Serial.println(F("ms (목표 수렴)"));
    stopAllPumps();
    closeAllValves();
    activeState = AS_LOCKED;
    activeStateEnteredMs = millis();
  }
}

void runActiveMode() {
  switch (activeState) {
    case AS_RESET: tickReset(); break;
    case AS_INFLATE: tickInflate(); break;
    case AS_SWEEP: tickSweep(); break;
    case AS_RETURN: tickReturn(); break;
    case AS_LOCKED: readAllSensors(); break;
  }
}

// ================= 모드1(AUTO FIT) 캘리브레이션 UI =================
enum CalState { CAL_IDLE, CAL_ANALYZING, CAL_COMPLETE_SHOW, CAL_LOCKED };
CalState calState = CAL_IDLE;
unsigned long calStateStartMs = 0;
unsigned long lastLetterMs = 0;
uint8_t revealedChars = 0;

const char LINE0_TEXT[] = "Analyzing your";
const char LINE1_TEXT[] = "body pressure";
const uint8_t LINE0_LEN = sizeof(LINE0_TEXT) - 1;
const uint8_t LINE1_LEN = sizeof(LINE1_TEXT) - 1;
const uint8_t ANALYZE_TOTAL_LEN = LINE0_LEN + LINE1_LEN;
const unsigned long LETTER_REVEAL_MS = 150;

const unsigned long DOT_ANIM_INTERVAL_MS = 400;
unsigned long lastDotAnimMs = 0;
uint8_t dotCount = 1;

void drawAnalyzingPartial(uint8_t count) {
  char line0[17];
  char line1[17];
  for (uint8_t i = 0; i < 16; i++) { line0[i] = ' '; line1[i] = ' '; }
  line0[16] = '\0';
  line1[16] = '\0';
  for (uint8_t i = 0; i < count && i < ANALYZE_TOTAL_LEN; i++) {
    if (i < LINE0_LEN) line0[i] = LINE0_TEXT[i];
    else line1[i - LINE0_LEN] = LINE1_TEXT[i - LINE0_LEN];
  }
  lcdPrintLine(0, line0);
  lcdPrintLine(1, line1);
}

void drawDotAnimFrame() {
  char line1[17];
  uint8_t i = 0;
  for (; i < LINE1_LEN; i++) line1[i] = LINE1_TEXT[i];
  for (uint8_t d = 0; d < dotCount && i < 16; d++, i++) line1[i] = '.';
  for (; i < 16; i++) line1[i] = ' ';
  line1[16] = '\0';
  lcdPrintLine(1, line1);
}

void playCompleteChime() {
  const int notes[4] = {262, 330, 392, 523}; // 도-미-솔-도
  for (uint8_t i = 0; i < 4; i++) {
    tone(BUZZER_PIN, notes[i]);
    delay(100);
  }
  noTone(BUZZER_PIN);
}

void showMaintainingPressure() {
  lcdPrintLine(0, "Maintaining");
  lcdPrintLine(1, "Optimal Pressure");
}

void startCalibration() {
  calState = CAL_ANALYZING;
  calStateStartMs = millis();
  lastLetterMs = 0;
  revealedChars = 0;
  lastDotAnimMs = 0;
  dotCount = 1;
  lcdClear();
  enterReset(); // 현재 상태와 무관하게 일단 전부 배기하고 시작
  Serial.println(F("캘리브레이션(능동형) 시작 - 초기화(배기)부터"));
}

void tickCalibration() {
  if (calState == CAL_IDLE || calState == CAL_LOCKED) return;

  unsigned long now = millis();

  if (calState == CAL_ANALYZING) {
    runActiveMode(); // 실제 밸브/펌프/센서 진행

    if (revealedChars < ANALYZE_TOTAL_LEN) {
      if (now - lastLetterMs >= LETTER_REVEAL_MS) {
        lastLetterMs = now;
        revealedChars++;
        drawAnalyzingPartial(revealedChars);
      }
    } else if (now - lastDotAnimMs >= DOT_ANIM_INTERVAL_MS) {
      lastDotAnimMs = now;
      drawDotAnimFrame();
      dotCount++;
      if (dotCount > 3) dotCount = 1;
    }

    if (activeState == AS_LOCKED) {
      calState = CAL_COMPLETE_SHOW;
      calStateStartMs = now;
      lcdPrintLine(0, "Analyze Complete");
      lcdPrintLine(1, "!");
      playCompleteChime();
      Serial.println(F("캘리브레이션 완료"));
    }
  } else if (calState == CAL_COMPLETE_SHOW) {
    if (now - calStateStartMs >= 2000) {
      calState = CAL_LOCKED;
      showMaintainingPressure();
      Serial.println(F("잠금 모드 진입"));
    }
  }
}

// ================= 모드2(REALTIME) 자세 보정 =================
enum Mode2State { M2_IDLE, M2_BALANCED, M2_ALERT };
Mode2State mode2State = M2_IDLE;
unsigned long alertStartMs = 0;
unsigned long lastBlinkMs = 0;
bool blinkOn = false;
const unsigned long ALERT_DURATION_MS = 10000; // 자세 복귀가 계속 안 될 경우의 최대 유지 시간(안전상한)
const unsigned long BLINK_INTERVAL_MS = 250;
const unsigned int ALERT_TONE_HZ = 440;
uint8_t alertBeepCount = 0;
const uint8_t ALERT_BEEP_MAX = 4;

// 실측 센서 기반 자세 이상 감지: BALANCED 진입 시점의 압력을 기준압으로 저장해두고,
// 각 포켓이 기준압 대비 postureThreshold 이상 벗어나면 자세가 흐트러진 것으로 판단
uint16_t postureBaseline[4];
uint16_t postureThreshold = 40; // raw 편차 임계값 (시리얼 POSTURE,<raw> 로 조정 가능)
const unsigned long POSTURE_CHECK_INTERVAL_MS = 200;
unsigned long lastPostureCheckMs = 0;
const unsigned long ALERT_MIN_MS = 1000; // 짧은 순간 튐으로 바로 복귀 판정되지 않도록 최소 유지 시간

// 자세이상이 순간적인 튐이 아니라 일정 시간 지속될 때만 알림 (2026-07-29: 너무 빨리 울린다는 피드백으로 추가)
unsigned long postureConfirmMs = 2000; // 시리얼 POSTURECONFIRM,<ms> 로 조정 가능
bool imbalanceActive = false;
unsigned long imbalanceSinceMs = 0;

bool postureImbalanced() {
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t dev = sensorRaw[i] > postureBaseline[i]
                     ? sensorRaw[i] - postureBaseline[i]
                     : postureBaseline[i] - sensorRaw[i];
    if (dev > postureThreshold) return true;
  }
  return false;
}

void showWellBalanced() {
  lcdPrintLine(0, "Well balanced :)");
  lcdPrintLine(1, "");
}

void showSitUpStraight() {
  lcdPrintLine(0, "Sit up straight");
  lcdPrintLine(1, "!!!");
}

void resetMode2() {
  mode2State = M2_IDLE;
  noTone(BUZZER_PIN);
  lcdBacklightOn();
}

void enterMode2Balanced() {
  mode2State = M2_BALANCED;
  noTone(BUZZER_PIN);
  lcdBacklightOn();
  stopAllPumps();
  closeAllValves(); // 진입 시점 경도(밸브 잠금) 상태 유지
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) postureBaseline[i] = sensorRaw[i];
  lastPostureCheckMs = millis();
  imbalanceActive = false;
  showWellBalanced();
  Serial.println(F("모드2 진입 - Well balanced (기준압 저장)"));
}

void enterMode2Alert() {
  mode2State = M2_ALERT;
  alertStartMs = millis();
  lastBlinkMs = 0;
  blinkOn = false;
  alertBeepCount = 0;
  showSitUpStraight();
  // ---- 준비: 원하는 자세로 유도하는 실제 액추에이션(밸브/펌프 조정) 로직 필요 - 트리거/보정 알고리즘 실험 후 구현
  Serial.println(F("자세 이상 감지 - Sit up straight"));
}

void tickMode2() {
  if (mode2State == M2_IDLE) return;

  unsigned long now = millis();

  if (mode2State == M2_BALANCED) {
    if (now - lastPostureCheckMs >= POSTURE_CHECK_INTERVAL_MS) {
      lastPostureCheckMs = now;
      if (postureImbalanced()) {
        if (!imbalanceActive) {
          imbalanceActive = true;
          imbalanceSinceMs = now;
        } else if (now - imbalanceSinceMs >= postureConfirmMs) {
          enterMode2Alert();
          return;
        }
      } else {
        imbalanceActive = false;
      }
    }
    // 자세가 안정적일 땐 엔코더를 돌려도 반응하지 않도록 잔여 회전을 흘려버림
    noInterrupts();
    rotAccum = 0;
    interrupts();
  }

  if (mode2State == M2_ALERT) {
    if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      if (blinkOn) {
        lcdBacklightOn();
        if (alertBeepCount < ALERT_BEEP_MAX) {
          tone(BUZZER_PIN, ALERT_TONE_HZ);
          alertBeepCount++;
        }
      } else {
        lcdBacklightOff();
        noTone(BUZZER_PIN);
      }
    }
    unsigned long elapsed = now - alertStartMs;
    bool recovered = elapsed >= ALERT_MIN_MS && !postureImbalanced();
    if (recovered || elapsed >= ALERT_DURATION_MS) {
      noTone(BUZZER_PIN);
      lcdBacklightOn();
      playCompleteChime();
      enterMode2Balanced();
    }
  }
}

// ================= 모드4(REALTIME-ACTIVE) 실시간 능동보정 프로토타입 =================
// 기본 감지 로직은 모드2와 동일(postureBaseline/postureThreshold/postureConfirmMs 등 재사용,
// 모드2와 모드4는 동시에 돌지 않으므로 상태 공유해도 안전함). 모드2와 다른 점은 감지 이후
// 그냥 알림만 하고 끝나는 게 아니라, 압력이 오른(=하중이 더 실린) 포켓들을 상한까지 채워서
// 눌러주는 능동 보정 동작까지 수행한다는 것.
enum Mode4State { M4_IDLE, M4_MONITORING, M4_ALERT, M4_BOOSTING, M4_HOLD, M4_RETURNING };
Mode4State mode4State = M4_IDLE;
unsigned long mode4AlertStartMs = 0;
unsigned long mode4HoldStartMs = 0;
bool mode4BoostPocket[4] = {false, false, false, false};
unsigned long mode4AlertShowMs = 2000;  // 알림(점멸/비프) 표시 시간, 시리얼 MODE4ALERT,<ms> 로 조정
unsigned long mode4HoldMs = 3000;       // 다 채운 뒤 복귀 전 대기시간, 시리얼 MODE4HOLD,<ms> 로 조정

void enterMode4Monitoring() {
  mode4State = M4_MONITORING;
  noTone(BUZZER_PIN);
  lcdBacklightOn();
  stopAllPumps();
  closeAllValves();
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) postureBaseline[i] = sensorRaw[i];
  lastPostureCheckMs = millis();
  imbalanceActive = false;
  showWellBalanced();
  Serial.println(F("모드4 진입 - 실시간 능동보정 대기 (기준압 저장)"));
}

void enterMode4Alert() {
  mode4State = M4_ALERT;
  mode4AlertStartMs = millis();
  lastBlinkMs = 0;
  blinkOn = false;
  alertBeepCount = 0;
  showSitUpStraight();
  Serial.println(F("모드4 - 자세이상 감지, 보정 준비"));
}

void enterMode4Boosting() {
  mode4State = M4_BOOSTING;
  readAllSensors();
  bool any = false;
  for (uint8_t i = 0; i < 4; i++) {
    mode4BoostPocket[i] = sensorRaw[i] > postureBaseline[i] &&
                          (uint16_t)(sensorRaw[i] - postureBaseline[i]) > postureThreshold;
    if (mode4BoostPocket[i]) any = true;
  }
  if (!any) {
    // 방어: 이론상 여기 오면 최소 하나는 있어야 하지만, 혹시 몰라 대기 상태로 되돌림
    enterMode4Monitoring();
    return;
  }
  for (uint8_t g = 0; g < 2; g++) {
    bool groupNeeded = mode4BoostPocket[g * 2] || mode4BoostPocket[g * 2 + 1];
    setValve3way(g, groupNeeded);
    if (groupNeeded) runPump(g, pumpSpeed); else stopPump(g);
  }
  for (uint8_t i = 0; i < 4; i++) setValve2way(i, mode4BoostPocket[i]);
  lcdPrintLine(0, "Adjusting...");
  lcdPrintLine(1, "");
  Serial.println(F("모드4 - 압력 상승 포켓 채우기 시작"));
}

void tickMode4Boosting() {
  readAllSensors();
  bool allDone = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (mode4BoostPocket[i]) {
      if (sensorRaw[i] >= pressureCeilingRaw) {
        setValve2way(i, false);
        mode4BoostPocket[i] = false;
      } else {
        allDone = false;
      }
    }
  }
  for (uint8_t g = 0; g < 2; g++) {
    bool stillNeeded = mode4BoostPocket[g * 2] || mode4BoostPocket[g * 2 + 1];
    if (!stillNeeded) {
      stopPump(g);
      setValve3way(g, false);
    }
  }
  if (allDone) {
    stopAllPumps();
    for (uint8_t g = 0; g < 2; g++) setValve3way(g, false);
    mode4State = M4_HOLD;
    mode4HoldStartMs = millis();
    lcdPrintLine(0, "Holding...");
    Serial.println(F("모드4 - 채우기 완료, 홀드 시작"));
  }
}

void tickMode4Hold() {
  if (millis() - mode4HoldStartMs >= mode4HoldMs) {
    mode4State = M4_RETURNING;
    startTargetSeek(postureBaseline);
    lcdPrintLine(0, "Returning...");
    Serial.println(F("모드4 - 이전 압력으로 복귀 시작"));
  }
}

void tickMode4Returning() {
  if (tickTargetSeek()) {
    stopAllPumps();
    closeAllValves();
    enterMode4Monitoring(); // 재무장(새 기준압 다시 캡처) - 청각 알림 없이 조용히 재시작
  }
}

void resetMode4() {
  mode4State = M4_IDLE;
  stopAllPumps();
  closeAllValves();
  noTone(BUZZER_PIN);
  lcdBacklightOn();
}

void tickMode4() {
  if (mode4State == M4_IDLE) return;

  unsigned long now = millis();

  if (mode4State == M4_MONITORING) {
    if (now - lastPostureCheckMs >= POSTURE_CHECK_INTERVAL_MS) {
      lastPostureCheckMs = now;
      if (postureImbalanced()) {
        if (!imbalanceActive) {
          imbalanceActive = true;
          imbalanceSinceMs = now;
        } else if (now - imbalanceSinceMs >= postureConfirmMs) {
          enterMode4Alert();
          return;
        }
      } else {
        imbalanceActive = false;
      }
    }
    noInterrupts();
    rotAccum = 0;
    interrupts();
  } else if (mode4State == M4_ALERT) {
    // 리얼타임 액티브는 압력으로 직접 교정해주는 모드라 청각 알림은 불필요 -
    // LCD 점멸은 그대로 두고 부저(비프)만 뺌(2026-08-05)
    if (now - lastBlinkMs >= BLINK_INTERVAL_MS) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      if (blinkOn) {
        lcdBacklightOn();
      } else {
        lcdBacklightOff();
      }
    }
    if (now - mode4AlertStartMs >= mode4AlertShowMs) {
      lcdBacklightOn();
      enterMode4Boosting();
    }
  } else if (mode4State == M4_BOOSTING) {
    tickMode4Boosting();
  } else if (mode4State == M4_HOLD) {
    tickMode4Hold();
  } else if (mode4State == M4_RETURNING) {
    tickMode4Returning();
  }
}

// ================= 모드5(STRETCHING) 대각선 포켓 교대 스트레칭 =================
// 순서: (전부 배기) -> 포켓0+3 채우고 3초 유지 후 배기 -> 포켓1+2 채우고 3초 유지 후 배기
// 이 한 세트를 stretchReps(기본 10)번 반복 -> 시작 전 압력으로 복귀 + 완료 알림
enum Mode5Sub { M5_IDLE, M5_RESET, M5_INFLATE, M5_HOLD, M5_DEFLATE, M5_RETURNING, M5_DONE };
Mode5Sub mode5Sub = M5_IDLE;
unsigned long mode5PhaseStartMs = 0;
uint16_t mode5PreviousRaw[4] = {0, 0, 0, 0}; // 스트레칭 시작 전 압력(끝나면 이 값으로 복귀)
bool stretchPhaseA = true;                   // true=포켓0,3 진행 중, false=포켓1,2 진행 중
uint8_t stretchRepCount = 0;                 // 완료된 세트 수(A+B 한 번 = 1세트)
uint8_t stretchReps = 10;                    // 시리얼 STRETCHREPS,<n> 로 조정
unsigned long stretchHoldMs = 3000;          // 시리얼 STRETCHHOLD,<ms> 로 조정
bool stretchDone[4] = {false, false, false, false};

bool stretchPocketActive(uint8_t i) {
  return stretchPhaseA ? (i == 0 || i == 3) : (i == 1 || i == 2);
}

void drawMode5Screen(const char *phaseLabel) {
  char line0[17];
  snprintf(line0, sizeof(line0), "Stretch %d/%d", stretchRepCount + 1, stretchReps);
  lcdPrintLine(0, line0);
  char line1[17];
  snprintf(line1, sizeof(line1), "%s P%d+P%d", phaseLabel, stretchPhaseA ? 1 : 2, stretchPhaseA ? 4 : 3);
  lcdPrintLine(1, line1);
}

void enterMode5Reset() {
  mode5Sub = M5_RESET;
  mode5PhaseStartMs = millis();
  stopAllPumps();
  setValve3way(0, false);
  setValve3way(1, false);
  for (uint8_t i = 0; i < 4; i++) setValve2way(i, true);
  lcdPrintLine(0, "Stretching");
  lcdPrintLine(1, "Preparing...");
  Serial.println(F("모드5 - 초기화(전체 배기) 시작"));
}

void tickMode5Reset() {
  readAllSensors();
  bool allVented = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (sensorRaw[i] > resetTargetRaw) allVented = false;
  }
  bool timedOut = (millis() - mode5PhaseStartMs) > RESET_TIMEOUT_MS;
  if (allVented || timedOut) {
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    stretchPhaseA = true;
    stretchRepCount = 0;
    enterMode5Inflate();
  }
}

void enterMode5Inflate() {
  mode5Sub = M5_INFLATE;
  mode5PhaseStartMs = millis();
  for (uint8_t i = 0; i < 4; i++) {
    bool active = stretchPocketActive(i);
    stretchDone[i] = !active; // 이번 페이즈에 관여 안 하는 포켓은 이미 완료 취급
    setValve2way(i, active);
  }
  for (uint8_t g = 0; g < 2; g++) {
    bool groupNeeded = stretchPocketActive(g * 2) || stretchPocketActive(g * 2 + 1);
    setValve3way(g, groupNeeded);
    if (groupNeeded) runPump(g, pumpSpeed); else stopPump(g);
  }
  drawMode5Screen("Fill");
  Serial.println(F("모드5 - 포켓 채우기 시작"));
}

void tickMode5Inflate() {
  readAllSensors();
  bool allDone = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (!stretchDone[i]) {
      if (sensorRaw[i] >= pressureCeilingRaw) {
        setValve2way(i, false);
        stretchDone[i] = true;
      } else {
        allDone = false;
      }
    }
  }
  for (uint8_t g = 0; g < 2; g++) {
    bool stillNeeded = (!stretchDone[g * 2]) || (!stretchDone[g * 2 + 1]);
    if (!stillNeeded) {
      stopPump(g);
      setValve3way(g, false);
    }
  }
  if (allDone) {
    stopAllPumps();
    for (uint8_t g = 0; g < 2; g++) setValve3way(g, false);
    mode5Sub = M5_HOLD;
    mode5PhaseStartMs = millis();
    drawMode5Screen("Hold");
    Serial.println(F("모드5 - 홀드 시작"));
  }
}

void tickMode5Hold() {
  if (millis() - mode5PhaseStartMs >= stretchHoldMs) {
    mode5Sub = M5_DEFLATE;
    mode5PhaseStartMs = millis();
    for (uint8_t i = 0; i < 4; i++) {
      bool active = stretchPocketActive(i);
      stretchDone[i] = !active;
      if (active) setValve2way(i, true); // 배기(대기모드에서 열기)
    }
    drawMode5Screen("Vent");
    Serial.println(F("모드5 - 배기 시작"));
  }
}

void tickMode5Deflate() {
  readAllSensors();
  bool allDone = true;
  for (uint8_t i = 0; i < 4; i++) {
    if (!stretchDone[i]) {
      if (sensorRaw[i] <= resetTargetRaw) {
        setValve2way(i, false);
        stretchDone[i] = true;
      } else {
        allDone = false;
      }
    }
  }
  bool timedOut = (millis() - mode5PhaseStartMs) > RESET_TIMEOUT_MS;
  if (allDone || timedOut) {
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    if (stretchPhaseA) {
      stretchPhaseA = false;
      enterMode5Inflate();
    } else {
      stretchPhaseA = true;
      stretchRepCount++;
      if (stretchRepCount >= stretchReps) {
        mode5Sub = M5_RETURNING;
        startTargetSeek(mode5PreviousRaw);
        lcdPrintLine(0, "Stretch Done");
        lcdPrintLine(1, "Returning...");
        Serial.println(F("모드5 - 스트레칭 반복 완료, 이전 압력으로 복귀 시작"));
      } else {
        enterMode5Inflate();
      }
    }
  }
}

void tickMode5Returning() {
  if (tickTargetSeek()) {
    stopAllPumps();
    closeAllValves();
    playCompleteChime();
    mode5Sub = M5_DONE;
    lcdPrintLine(0, "Stretching");
    lcdPrintLine(1, "Complete! :)");
    Serial.println(F("모드5 - 완료"));
  }
}

void enterMode5() {
  mode5Sub = M5_IDLE;
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) mode5PreviousRaw[i] = sensorRaw[i];
  enterMode5Reset();
}

void resetMode5() {
  mode5Sub = M5_IDLE;
  stopAllPumps();
  closeAllValves();
}

void tickMode5() {
  if (mode5Sub == M5_IDLE || mode5Sub == M5_DONE) return;

  if (mode5Sub == M5_RESET) {
    tickMode5Reset();
  } else if (mode5Sub == M5_INFLATE) {
    tickMode5Inflate();
  } else if (mode5Sub == M5_HOLD) {
    tickMode5Hold();
  } else if (mode5Sub == M5_DEFLATE) {
    tickMode5Deflate();
  } else if (mode5Sub == M5_RETURNING) {
    tickMode5Returning();
  }
}

// ================= 모드3(MANUAL) 포켓 선택/조절 + 프리셋 =================
enum Mode3State { M3_IDLE, M3_SELECT, M3_ADJUST, M3_PRESET_SELECT, M3_PRESET_SAVED };
Mode3State mode3State = M3_IDLE;
uint8_t selectedPocket = 0; // 0~3 = 포켓 1~4, 4 = 북마크(프리셋 저장)
uint8_t presetSelectIdx = 0; // 0~2 = 프리셋 1~3
uint8_t presets[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // 프리셋별 저장된 포켓 레벨(0~10, 0=완전배기 특수값), EEPROM과 동기화
const int EEPROM_PRESET_BASE = 0; // 프리셋 3개 * 4바이트 = 12바이트(주소 0~11) 사용
const int EEPROM_SEED_FLAG_ADDR = EEPROM_PRESET_BASE + 12; // 주소 12: 기본 프리셋 시딩 여부 마커
const uint8_t EEPROM_SEED_MAGIC = 0xA6; // 값 바뀔 때마다 이 매직바이트를 올려서 재시딩 트리거 (2026-07-28: 0xA5->0xA6)

void loadPresetsFromEeprom() {
  for (uint8_t p = 0; p < 3; p++) {
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t v = EEPROM.read(EEPROM_PRESET_BASE + p * 4 + i);
      presets[p][i] = (v <= 10) ? v : 0;
    }
  }
}

void savePresetToEeprom(uint8_t idx) {
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_PRESET_BASE + idx * 4 + i, presets[idx][i]);
  }
}

// 최초 1회(EEPROM에 시딩 마커가 없을 때)만 기본 프리셋 값을 심어둠 -
// 이후에는 사용자가 북마크로 저장한 값이 계속 유지되고 다시 덮어쓰지 않음
void seedDefaultPresetsIfNeeded() {
  if (EEPROM.read(EEPROM_SEED_FLAG_ADDR) == EEPROM_SEED_MAGIC) return;
  const uint8_t defaults[3][4] = {{1, 1, 8, 8}, {2, 2, 2, 2}, {8, 8, 8, 8}};
  for (uint8_t p = 0; p < 3; p++) {
    for (uint8_t i = 0; i < 4; i++) {
      presets[p][i] = defaults[p][i];
    }
    savePresetToEeprom(p);
  }
  EEPROM.update(EEPROM_SEED_FLAG_ADDR, EEPROM_SEED_MAGIC);
  Serial.println(F("기본 프리셋 시딩 완료: 1=1,1,8,8 / 2=2,2,2,2 / 3=8,8,8,8"));
}

unsigned long lastBlinkSelMs = 0;
bool blinkSelOn = true;
const unsigned long SELECT_BLINK_MS = 500;

// 매뉴얼 조절: "계속 돌리는 동안만" 홀드 방식 (파일럿2와 동일 패턴)
unsigned long adjustHoldTimeoutMs = 250; // 시리얼 HOLD,<ms> 명령으로 조정 (50~1000 범위로 clamp)
const unsigned long ADJUST_HOLD_MIN_MS = 50;
const unsigned long ADJUST_HOLD_MAX_MS = 1000;
unsigned long lastAdjustCwMs = 0;
unsigned long lastAdjustCcwMs = 0;
unsigned long lastAdjustDrawMs = 0;
const unsigned long ADJUST_DRAW_INTERVAL_MS = 150;
int8_t pendingAdjustDir = 0;   // 파일럿2와 동일한 "첫 펄스 확인" 패턴 - 단발성 신호는 무시
unsigned long pendingAdjustMs = 0;
int8_t lastAdjustAction = 0;   // 0=정지, 1=부풀리기, -1=배기 (직전 상태 - 바뀔 때만 I2C 씀)

void showMode3Default() {
  readAllSensors();
  lcdPrintLine(0, "MODE 2 - MANUAL");
  char line1[17];
  snprintf(line1, sizeof(line1), "%d %d %d %d",
           rawToLevel(sensorRaw[0]), rawToLevel(sensorRaw[1]),
           rawToLevel(sensorRaw[2]), rawToLevel(sensorRaw[3]));
  lcdPrintLine(1, line1);
}

void drawSelectScreen() {
  lcdPrintLine(0, "select section");
  char line1[17];
  uint8_t pos = 0;
  for (uint8_t i = 0; i < 4; i++) {
    bool hide = (i == selectedPocket) && !blinkSelOn;
    line1[pos++] = hide ? ' ' : ('1' + i);
    line1[pos++] = ' ';
    line1[pos++] = ' ';
  }
  bool hideBookmark = (selectedPocket == 4) && !blinkSelOn;
  line1[pos++] = hideBookmark ? ' ' : (char)CHAR_BOOKMARK;
  line1[pos] = '\0';
  lcdPrintLine(1, line1);
}

void drawAdjustScreen() {
  char line0[17];
  snprintf(line0, sizeof(line0), "#%d POCKET Adj.", selectedPocket + 1);
  lcdPrintLine(0, line0);
  char line1[17];
  snprintf(line1, sizeof(line1), "Level %d", rawToLevel(sensorRaw[selectedPocket]));
  lcdPrintLine(1, line1);
}

void drawPresetSelectScreen() {
  lcdPrintLine(0, "PRESET");
  char line1[17];
  uint8_t pos = 0;
  for (uint8_t i = 0; i < 3; i++) {
    bool hide = (i == presetSelectIdx) && !blinkSelOn;
    line1[pos++] = hide ? ' ' : ('1' + i);
    if (i < 2) line1[pos++] = ' ';
  }
  line1[pos] = '\0';
  lcdPrintLine(1, line1);
}

void drawPresetSaved() {
  char line0[17];
  snprintf(line0, sizeof(line0), "PRESET #%d", presetSelectIdx + 1);
  lcdPrintLine(0, line0);
  lcdPrintLine(1, "Saved! :)");
}

void savePresetFromCurrent(uint8_t idx) {
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) presets[idx][i] = rawToLevel(sensorRaw[i]);
  savePresetToEeprom(idx);
}

void resetMode3() {
  mode3State = M3_IDLE;
}

void enterMode3Select() {
  mode3State = M3_SELECT;
  selectedPocket = 0;
  blinkSelOn = true;
  lastBlinkSelMs = millis();
  drawSelectScreen();
}

void enterMode3Adjust() {
  mode3State = M3_ADJUST;
  lastAdjustCwMs = millis() - adjustHoldTimeoutMs - 1;
  lastAdjustCcwMs = millis() - adjustHoldTimeoutMs - 1;
  pendingAdjustDir = 0;
  lastAdjustAction = 0;
  readAllSensors();
  drawAdjustScreen();
}

void enterPresetSelect() {
  mode3State = M3_PRESET_SELECT;
  presetSelectIdx = 0;
  blinkSelOn = true;
  lastBlinkSelMs = millis();
  drawPresetSelectScreen();
}

void tickMode3() {
  if (mode3State == M3_IDLE) return;

  unsigned long now = millis();

  noInterrupts();
  int8_t step = rotAccum;
  rotAccum = 0;
  interrupts();

  if (mode3State == M3_SELECT) {
    // step의 부호만 보고 항상 1칸만 움직이면, LCD 갱신 등으로 루프가 잠깐 지연되는 동안
    // 같은 방향 펄스가 여러 개 쌓였을 때 그 중 일부가 씹힌 것처럼 사라짐 - 누적된
    // 만큼(step 크기 그대로) 반영하고 범위 안으로 클램프
    if (step != 0) {
      int newSel = (int)selectedPocket + step;
      if (newSel < 0) newSel = 0;
      if (newSel > 4) newSel = 4;
      if (newSel != (int)selectedPocket) {
        selectedPocket = (uint8_t)newSel;
        blinkSelOn = true;
        lastBlinkSelMs = now;
        drawSelectScreen();
      }
    }
    if (now - lastBlinkSelMs >= SELECT_BLINK_MS) {
      lastBlinkSelMs = now;
      blinkSelOn = !blinkSelOn;
      drawSelectScreen();
    }
  } else if (mode3State == M3_ADJUST) {
    // 파일럿2와 동일한 "첫 펄스 확인" 홀드 방식: 단발성 회전 신호는 바로 반영하지 않고
    // pending으로만 기록해두고, 유효시간 내에 같은 방향 신호가 한 번 더 들어오거나
    // 이미 홀드 중이면 그때 실제로 반영한다. 회전량 자체는 그대로 누적되므로 조절 정밀도는
    // 그대로고, 릴레이/솔레노이드에 순간적으로 튀는 단발 신호만 걸러진다.
    if (step > 0) {
      bool alreadyHoldingCw = (now - lastAdjustCwMs) < adjustHoldTimeoutMs;
      bool pendingConfirmed = pendingAdjustDir == 1 && (now - pendingAdjustMs) < adjustHoldTimeoutMs;
      if (alreadyHoldingCw || pendingConfirmed) {
        lastAdjustCwMs = now;
        pendingAdjustDir = 0;
      } else {
        pendingAdjustDir = 1;
        pendingAdjustMs = now;
      }
    } else if (step < 0) {
      bool alreadyHoldingCcw = (now - lastAdjustCcwMs) < adjustHoldTimeoutMs;
      bool pendingConfirmed = pendingAdjustDir == -1 && (now - pendingAdjustMs) < adjustHoldTimeoutMs;
      if (alreadyHoldingCcw || pendingConfirmed) {
        lastAdjustCcwMs = now;
        pendingAdjustDir = 0;
      } else {
        pendingAdjustDir = -1;
        pendingAdjustMs = now;
      }
    }
    if (pendingAdjustDir != 0 && (now - pendingAdjustMs) >= adjustHoldTimeoutMs) {
      pendingAdjustDir = 0; // 시간 내에 확인되지 않은 단발 신호는 폐기
    }

    bool holdingCw = (now - lastAdjustCwMs) < adjustHoldTimeoutMs;
    bool holdingCcw = (now - lastAdjustCcwMs) < adjustHoldTimeoutMs;

    uint8_t groupIdx = POCKET_GROUP[selectedPocket];
    readAllSensors();

    // 상/하한 경계 근처에서 센서 노이즈(±toleranceRaw)만으로 on/off가 계속 뒤집히는 걸
    // 막기 위해 여유폭을 둠 (안 그러면 경계에서 릴레이가 초고속으로 딱딱거릴 수 있음)
    bool shouldInflate = holdingCw && sensorRaw[selectedPocket] < (pressureCeilingRaw - toleranceRaw);
    bool shouldDeflate = !shouldInflate && holdingCcw && sensorRaw[selectedPocket] > (pressureFloorRaw + toleranceRaw);

    int8_t desiredAction = shouldInflate ? 1 : (shouldDeflate ? -1 : 0);
    if (desiredAction != lastAdjustAction) {
      if (desiredAction == 1) {
        setValve3way(groupIdx, true);
        setValve2way(selectedPocket, true);
        runPump(groupIdx, pumpSpeed);
      } else if (desiredAction == -1) {
        setValve3way(groupIdx, false);
        setValve2way(selectedPocket, true);
        stopPump(groupIdx);
      } else {
        setValve2way(selectedPocket, false);
        stopPump(groupIdx);
      }
      lastAdjustAction = desiredAction;
    }

    if (now - lastAdjustDrawMs >= ADJUST_DRAW_INTERVAL_MS) {
      lastAdjustDrawMs = now;
      drawAdjustScreen();
    }
  } else if (mode3State == M3_PRESET_SELECT) {
    if (step != 0) {
      int newSel = (int)presetSelectIdx + step;
      if (newSel < 0) newSel = 0;
      if (newSel > 2) newSel = 2;
      if (newSel != (int)presetSelectIdx) {
        presetSelectIdx = (uint8_t)newSel;
        blinkSelOn = true;
        lastBlinkSelMs = now;
        drawPresetSelectScreen();
      }
    }
    if (now - lastBlinkSelMs >= SELECT_BLINK_MS) {
      lastBlinkSelMs = now;
      blinkSelOn = !blinkSelOn;
      drawPresetSelectScreen();
    }
  }
}

// ================= 최상위: 모드 선택 (회전=미리보기 전환, 싱글클릭=확정) =================
enum TopState { TOP_MODE_SELECT, TOP_IN_MODE };
TopState topState = TOP_MODE_SELECT;
uint8_t modeSelectIdx = 0;
const char *MODE_NAMES[5] = {"AUTO FIT", "MANUAL", "REALTIME", "RT-ACTIVE", "STRETCHING"};

void showSystemOn() {
  lcdPrintLine(0, "SYSTEM ON");
  lcdPrintLine(1, "HELLO :)");
}

void showMode(uint8_t idx) {
  char line0[17];
  snprintf(line0, sizeof(line0), "MODE %d", idx + 1);
  lcdPrintLine(0, line0);
  lcdPrintLine(1, MODE_NAMES[idx]);
}

void showSystemOff() {
  lcdPrintLine(0, "SYSTEM OFF");
  lcdPrintLine(1, "GOOD BYE :)");
  delay(1200);
  lcdClear();
  lcdBacklightOff();
}

void showModeSelectPreview() {
  if (modeSelectIdx == 1) {
    showMode3Default();
  } else {
    showMode(modeSelectIdx);
  }
}

void enterModeSelect() {
  topState = TOP_MODE_SELECT;
  if (currentModeIdx >= 0 && currentModeIdx <= 4) modeSelectIdx = currentModeIdx;
  calState = CAL_IDLE;
  resetMode2();
  resetMode3();
  resetMode4();
  resetMode5();
  showModeSelectPreview();
}

void confirmModeSelect() {
  currentModeIdx = modeSelectIdx;
  topState = TOP_IN_MODE;
  if (currentModeIdx == 0) {
    startCalibration();
  } else if (currentModeIdx == 1) {
    enterMode3Select();
  } else if (currentModeIdx == 2) {
    enterMode2Balanced();
  } else if (currentModeIdx == 3) {
    enterMode4Monitoring();
  } else if (currentModeIdx == 4) {
    enterMode5();
  }
}

void tickModeSelect() {
  noInterrupts();
  int8_t step = rotAccum;
  rotAccum = 0;
  interrupts();

  bool changed = false;
  if (step != 0) {
    int newIdx = (int)modeSelectIdx + step;
    if (newIdx < 0) newIdx = 0;
    if (newIdx > 4) newIdx = 4;
    if (newIdx != (int)modeSelectIdx) {
      modeSelectIdx = (uint8_t)newIdx;
      changed = true;
    }
  }

  if (changed) showModeSelectPreview();
}

// ================= 싱글클릭/더블클릭 동작 분기 =================
void handleSingleClick() {
  if (topState == TOP_MODE_SELECT) {
    confirmModeSelect();
    return;
  }
  if (currentModeIdx == 0 && calState == CAL_IDLE) {
    startCalibration();
  } else if (currentModeIdx == 2 && mode2State == M2_IDLE) {
    enterMode2Balanced();
  } else if (currentModeIdx == 3 && mode4State == M4_IDLE) {
    enterMode4Monitoring();
  } else if (currentModeIdx == 4 && mode5Sub == M5_IDLE) {
    enterMode5();
  } else if (currentModeIdx == 1) {
    if (mode3State == M3_IDLE) {
      enterMode3Select();
    } else if (mode3State == M3_SELECT) {
      if (selectedPocket == 4) {
        enterPresetSelect();
      } else {
        enterMode3Adjust();
      }
    } else if (mode3State == M3_PRESET_SELECT) {
      savePresetFromCurrent(presetSelectIdx);
      mode3State = M3_PRESET_SAVED;
      drawPresetSaved();
    }
  }
}

void handleDoubleClick() {
  if (topState == TOP_MODE_SELECT) return;

  if (currentModeIdx == 0) {
    stopEverythingSafely();
    buzzerActive = false;
    enterModeSelect();
  } else if (currentModeIdx == 1) {
    if (mode3State == M3_SELECT) {
      enterModeSelect();
    } else {
      if (mode3State == M3_ADJUST) {
        // 홀드 도중(회전 중) 빠져나가도 액추에이터가 방치되지 않도록 명시적으로 정지
        uint8_t groupIdx = POCKET_GROUP[selectedPocket];
        stopPump(groupIdx);
        setValve2way(selectedPocket, false);
      }
      mode3State = M3_SELECT;
      blinkSelOn = true;
      lastBlinkSelMs = millis();
      drawSelectScreen();
    }
  } else if (currentModeIdx == 2) {
    enterModeSelect();
  } else if (currentModeIdx == 3) {
    // 어느 하위 상태(BOOSTING/HOLD/RETURNING 포함)에서 빠져나가든 하드웨어를 안전하게 정지
    stopEverythingSafely();
    enterModeSelect();
  } else if (currentModeIdx == 4) {
    // 어느 하위 단계(채우기/홀드/배기/복귀 포함)에서 빠져나가든 하드웨어를 안전하게 정지
    stopEverythingSafely();
    enterModeSelect();
  }
}

void tickClickResolution() {
  if (presetApplying) return; // 프리셋 적용(하드웨어 이동) 중엔 클릭 처리 보류 - 능동형 모드 등과 상태 충돌 방지
  if (clickPending && (millis() - clickPendingMs) > DOUBLE_CLICK_WINDOW_MS) {
    clickPending = false;
    handleSingleClick();
  }
}

void handlePowerSwitch() {
  bool swNow = digitalRead(ENC_SW_PIN);
  unsigned long now = millis();

  if (swNow == LOW && encSwPrevState == LOW && !longPressFired &&
      (now - encSwPressStartMs) >= LONG_PRESS_MS) {
    longPressFired = true;
    systemOn = !systemOn;
    Serial.print(F("시스템 "));
    Serial.println(systemOn ? F("ON") : F("OFF"));
    if (systemOn) {
      showSystemOn();
      delay(1200);
      enterModeSelect();
    } else {
      setLedExclusive(-1);
      stopEverythingSafely();
      buzzerActive = false;
      calState = CAL_IDLE;
      resetMode2();
      resetMode3();
      clickPending = false;
      presetApplying = false;
      showSystemOff();
    }
  }

  if (swNow == LOW && encSwPrevState == HIGH) {
    encSwPressStartMs = now;
    longPressFired = false;
  } else if (swNow == HIGH && encSwPrevState == LOW) {
    if (!longPressFired && systemOn) {
      if (presetApplying) {
        abortPresetApplying(); // 적용/복귀 도중 클릭하면 그 자리에서 바로 중단
      } else if (presetFrozen) {
        presetFrozen = false;
        enterModeSelect(); // 프리셋 적용 완료 화면에서 클릭하면 모드선택으로
      } else if (clickPending && (now - clickPendingMs) <= DOUBLE_CLICK_WINDOW_MS) {
        clickPending = false;
        handleDoubleClick();
      } else {
        clickPending = true;
        clickPendingMs = now;
      }
    }
  }
  encSwPrevState = swNow;
}

// ================= 프리셋 전역 단축키 (택트스위치, 모든 화면에서 진입 가능) =================

void activatePreset(uint8_t idx) {
  bool anySaved = false;
  for (uint8_t i = 0; i < 4; i++) if (presets[idx][i] != 0) anySaved = true;

  setLedExclusive(idx);
  lcdBacklightOn();
  char line0[17];
  snprintf(line0, sizeof(line0), "PRESET #%d", idx + 1);
  lcdPrintLine(0, line0);

  if (!anySaved) {
    lcdPrintLine(1, "Empty");
    presetFrozen = true;
    Serial.print(F("프리셋 "));
    Serial.print(idx + 1);
    Serial.println(F(" 저장된 값 없음"));
    return;
  }

  lcdPrintLine(1, "Applying...");
  readAllSensors();
  for (uint8_t i = 0; i < 4; i++) presetPreviousRaw[i] = sensorRaw[i]; // 중단 시 되돌아갈 직전 상태 저장
  presetApplying = true;
  presetReverting = false;
  presetFrozen = false;
  presetApplyingIdx = idx;
  clickPending = false; // 적용 도중 대기 중이던 클릭이 나중에 엉뚱하게 발동하지 않도록 초기화
  uint16_t targets[4];
  for (uint8_t i = 0; i < 4; i++) targets[i] = levelToRawTarget(presets[idx][i]);
  startTargetSeek(targets);

  Serial.print(F("프리셋 "));
  Serial.print(idx + 1);
  Serial.print(F(" 적용 시작: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(presets[idx][i]);
    Serial.print(' ');
  }
  Serial.println();
}

// 프리셋 적용(또는 복귀) 도중 사용자가 다른 동작(클릭/다른 프리셋)을 시도하면
// 그 자리에서 즉시 중단하고, 목표를 "이 프리셋 적용 시작 직전 압력"으로 바꿔서
// 안전하게 되돌린다 - 이래야 4포켓이 어중간하게 섞인 상태로 방치되지 않는다.
void abortPresetApplying() {
  if (!presetApplying) return;
  presetReverting = true;
  startTargetSeek(presetPreviousRaw);
  lcdPrintLine(0, "Canceled");
  lcdPrintLine(1, "Reverting...");
  Serial.println(F("프리셋 적용 중단 - 직전 압력으로 복귀 시작"));
}

void handlePresetSwitches() {
  if (millis() - lastSwPollMs < SW_DEBOUNCE_MS) return;
  lastSwPollMs = millis();
  for (uint8_t i = 0; i < 3; i++) {
    bool pressed = readSwitch(i);
    if (pressed && !swWasPressed[i]) {
      if (calState == CAL_ANALYZING) {
        Serial.println(F("지금은 다른 하드웨어 동작 중이라 프리셋 적용 불가"));
      } else if (presetApplying) {
        abortPresetApplying();
      } else {
        activatePreset(i);
      }
    }
    swWasPressed[i] = pressed;
  }
}

// ================= 시리얼 명령 =================
void handleCommand(const String &line) {
  if (line.length() == 0) return;
  int firstComma = line.indexOf(',');
  String cmd = firstComma == -1 ? line : line.substring(0, firstComma);

  if (cmd == "CEIL") {
    pressureCeilingRaw = (uint16_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "FLOOR") {
    pressureFloorRaw = (uint16_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "TOL") {
    toleranceRaw = (uint16_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "SPEED") {
    pumpSpeed = (uint8_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "HOLD") {
    long v = line.substring(firstComma + 1).toInt();
    if (v < (long)ADJUST_HOLD_MIN_MS) v = ADJUST_HOLD_MIN_MS;
    if (v > (long)ADJUST_HOLD_MAX_MS) v = ADJUST_HOLD_MAX_MS;
    adjustHoldTimeoutMs = (unsigned long)v;
  } else if (cmd == "POSTURE") {
    postureThreshold = (uint16_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "RESETRAW") {
    resetTargetRaw = (uint16_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "POSTURECONFIRM") {
    postureConfirmMs = (unsigned long)line.substring(firstComma + 1).toInt();
  } else if (cmd == "MODE4ALERT") {
    mode4AlertShowMs = (unsigned long)line.substring(firstComma + 1).toInt();
  } else if (cmd == "MODE4HOLD") {
    mode4HoldMs = (unsigned long)line.substring(firstComma + 1).toInt();
  } else if (cmd == "STRETCHREPS") {
    stretchReps = (uint8_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "STRETCHHOLD") {
    stretchHoldMs = (unsigned long)line.substring(firstComma + 1).toInt();
  } else if (cmd == "DEBUGPRINT") {
    debugPrintEnabled = line.substring(firstComma + 1).toInt() != 0;
    Serial.print(F("DEBUGPRINT "));
    Serial.println(debugPrintEnabled ? F("ON") : F("OFF"));
  } else if (cmd == "ENCDEBUG") {
    encDebugEnabled = line.substring(firstComma + 1).toInt() != 0;
    Serial.print(F("ENCDEBUG "));
    Serial.println(encDebugEnabled ? F("ON") : F("OFF"));
  } else if (cmd == "PRESET") {
    // 택트스위치 미결선 상태에서 프리셋 적용을 시리얼로 대신 트리거 (PRESET,1~3, 사용자 표시번호 기준)
    int n = line.substring(firstComma + 1).toInt();
    if (n < 1 || n > 3) {
      Serial.println(F("PRESET,1~3 범위로 입력할 것"));
    } else if (!systemOn) {
      Serial.println(F("시스템이 꺼져있어서 프리셋 적용 불가 (롱프레스로 전원 켤 것)"));
    } else if (calState == CAL_ANALYZING || presetApplying) {
      Serial.println(F("지금은 다른 하드웨어 동작 중이라 프리셋 적용 불가"));
    } else {
      activatePreset((uint8_t)(n - 1));
    }
  } else if (cmd == "STATUS") {
    Serial.print(F("CEIL,"));
    Serial.print(pressureCeilingRaw);
    Serial.print(F(",FLOOR,"));
    Serial.print(pressureFloorRaw);
    Serial.print(F(",TOL,"));
    Serial.print(toleranceRaw);
    Serial.print(F(",SPEED,"));
    Serial.print(pumpSpeed);
    Serial.print(F(",HOLD,"));
    Serial.print(adjustHoldTimeoutMs);
    Serial.print(F(",POSTURE,"));
    Serial.print(postureThreshold);
    Serial.print(F(",RESETRAW,"));
    Serial.print(resetTargetRaw);
    Serial.print(F(",POSTURECONFIRM,"));
    Serial.print(postureConfirmMs);
    Serial.print(F(",MODE4ALERT,"));
    Serial.print(mode4AlertShowMs);
    Serial.print(F(",MODE4HOLD,"));
    Serial.print(mode4HoldMs);
    Serial.print(F(",STRETCHREPS,"));
    Serial.print(stretchReps);
    Serial.print(F(",STRETCHHOLD,"));
    Serial.print(stretchHoldMs);
    Serial.print(F(",DEBUGPRINT,"));
    Serial.print(debugPrintEnabled ? 1 : 0);
    Serial.print(F(",ENCDEBUG,"));
    Serial.println(encDebugEnabled ? 1 : 0);
  }
}

// ================= 디버그 시리얼 =================
void printDebug() {
  if (!debugPrintEnabled) return;
  if (millis() - lastDebugPrintMs < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMs = millis();
  Serial.print(F("TOP,"));
  Serial.print((int)topState);
  Serial.print(F(",MODE,"));
  Serial.print(currentModeIdx);
  Serial.print(F(",CAL,"));
  Serial.print((int)calState);
  Serial.print(F(",AS,"));
  Serial.print((int)activeState);
  Serial.print(F(",M2,"));
  Serial.print((int)mode2State);
  Serial.print(F(",M3,"));
  Serial.print((int)mode3State);
  Serial.print(F(",M4,"));
  Serial.print((int)mode4State);
  Serial.print(F(",M5,"));
  Serial.print((int)mode5Sub);
  Serial.print(F(",RAW,"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(sensorRaw[i]);
    Serial.print(i < 3 ? "," : "");
  }
  Serial.println();
}

// ================= setup/loop =================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  loadPresetsFromEeprom();
  seedDefaultPresetsIfNeeded();

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  encRawState = (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encoderISR, CHANGE);

  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  stopAllPumps();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pcfWrite(ADDR_VALVE, valveShadow);
  pcfWrite(ADDR_IO, ioShadow);

  lcdInit();
  lcdCreateChar(CHAR_BOOKMARK, BOOKMARK_GLYPH);
  lcdClear();
  lcdBacklightOff(); // 전원 OFF 상태 = LCD 꺼짐

  inputLine.reserve(64);
  Serial.println(F("파일럿3 시작 - 엔코더 롱프레스로 전원 ON/OFF"));
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      inputLine.trim();
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }

  tickEncDebug();
  handlePowerSwitch();
  tickClickResolution();

  if (systemOn) {
    readAllSensors(); // 어떤 화면/모드에 있든 sensorRaw는 항상 최신 상태로 유지 (디버그 RAW 출력이 특정 모드에서만 갱신되던 문제 수정)
    handlePresetSwitches();

    if (presetApplying) {
      if (tickTargetSeek()) {
        presetApplying = false;
        stopAllPumps();
        closeAllValves();
        if (presetReverting) {
          presetReverting = false;
          Serial.println(F("직전 압력으로 복귀 완료"));
          enterModeSelect();
        } else {
          char line1[17];
          snprintf(line1, sizeof(line1), "%d %d %d %d",
                   presets[presetApplyingIdx][0], presets[presetApplyingIdx][1],
                   presets[presetApplyingIdx][2], presets[presetApplyingIdx][3]);
          lcdPrintLine(1, line1);
          presetFrozen = true;
          Serial.println(F("프리셋 적용 완료 - 클릭할 때까지 화면 고정"));
        }
      }
    } else if (presetFrozen) {
      // 사용자가 클릭(모드선택으로 이동)하거나 다른 프리셋을 누를 때까지 이 화면 그대로 유지
    } else if (topState == TOP_MODE_SELECT) {
      tickModeSelect();
    } else {
      tickCalibration();
      tickMode2();
      tickMode3();
      tickMode4();
      tickMode5();
    }

    printDebug();
  }
}
