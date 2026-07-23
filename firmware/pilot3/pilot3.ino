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
// ✅ 이미 실측/확정된 것: LED_ACTIVE_LOW=false, IO익스팬더 0x20, 밸브익스팬더 0x24, LCD 핀배치(RS/RW/EN/BL/D4~D7), 엔코더 배선

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
unsigned long presetDisplayUntilMs = 0;
const unsigned long PRESET_DISPLAY_HOLD_MS = 1500;

// ---------- 엔코더 회전 ----------
volatile int8_t rotAccum = 0;
volatile unsigned long lastEncIsrMicros = 0;
const unsigned long ENC_ISR_DEBOUNCE_US = 5000;

void encoderISR() {
  unsigned long now = micros();
  if (now - lastEncIsrMicros < ENC_ISR_DEBOUNCE_US) return;
  lastEncIsrMicros = now;
  if (digitalRead(ENC_DT_PIN)) rotAccum--;
  else rotAccum++;
}

// ---------- IO/밸브 익스팬더 섀도 레지스터 ----------
uint8_t ioShadow = 0xFF;
uint8_t valveShadow = 0x00; // 기본값: 전부 비에너자이즈(안전)

unsigned long lastSwPollMs = 0;
bool swWasPressed[3] = {false, false, false};

String inputLine;
unsigned long lastDebugPrintMs = 0;

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
  if (level < 1) level = 1;
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
enum ActiveState { AS_INFLATE, AS_SWEEP, AS_RETURN, AS_LOCKED };
ActiveState activeState = AS_INFLATE;
unsigned long activeStateEnteredMs = 0;
unsigned long lastSweepTickMs = 0;

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
    stopAllPumps();
    closeAllValves();
    activeState = AS_LOCKED;
    activeStateEnteredMs = millis();
  }
}

void runActiveMode() {
  switch (activeState) {
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
  enterInflate(); // 실제 능동형 하드웨어 알고리즘 시작
  Serial.println(F("캘리브레이션(능동형) 시작"));
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
const unsigned long ALERT_DURATION_MS = 10000;
const unsigned long BLINK_INTERVAL_MS = 250;
const unsigned int ALERT_TONE_HZ = 440;
uint8_t alertBeepCount = 0;
const uint8_t ALERT_BEEP_MAX = 4;

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
  showWellBalanced();
  Serial.println(F("모드2 진입 - Well balanced"));
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

  // ---- 준비: 실제 자세 이상 감지 트리거로 교체할 것 (지금은 임시로 엔코더 CW 회전 감지)
  noInterrupts();
  int8_t step = rotAccum;
  rotAccum = 0;
  interrupts();
  if (mode2State == M2_BALANCED && step > 0) {
    enterMode2Alert();
    return;
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
    if (now - alertStartMs >= ALERT_DURATION_MS) { // ---- 준비 필요: 실측 조건(자세 복귀 판정)으로 교체
      noTone(BUZZER_PIN);
      lcdBacklightOn();
      playCompleteChime();
      enterMode2Balanced();
    }
  }
}

// ================= 모드3(MANUAL) 포켓 선택/조절 + 프리셋 =================
enum Mode3State { M3_IDLE, M3_SELECT, M3_ADJUST, M3_PRESET_SELECT, M3_PRESET_SAVED };
Mode3State mode3State = M3_IDLE;
uint8_t selectedPocket = 0; // 0~3 = 포켓 1~4, 4 = 북마크(프리셋 저장)
uint8_t presetSelectIdx = 0; // 0~2 = 프리셋 1~3
uint8_t presets[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // 프리셋별 저장된 포켓 레벨(1~10), EEPROM과 동기화
const int EEPROM_PRESET_BASE = 0; // 프리셋 3개 * 4바이트 = 12바이트(주소 0~11) 사용

void loadPresetsFromEeprom() {
  for (uint8_t p = 0; p < 3; p++) {
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t v = EEPROM.read(EEPROM_PRESET_BASE + p * 4 + i);
      presets[p][i] = (v >= 1 && v <= 10) ? v : 0;
    }
  }
}

void savePresetToEeprom(uint8_t idx) {
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_PRESET_BASE + idx * 4 + i, presets[idx][i]);
  }
}

unsigned long lastBlinkSelMs = 0;
bool blinkSelOn = true;
const unsigned long SELECT_BLINK_MS = 500;

// 매뉴얼 조절: "계속 돌리는 동안만" 홀드 방식 (파일럿2와 동일 패턴)
const unsigned long ADJUST_HOLD_TIMEOUT_MS = 200;
unsigned long lastAdjustCwMs = 0;
unsigned long lastAdjustCcwMs = 0;
unsigned long lastAdjustDrawMs = 0;
const unsigned long ADJUST_DRAW_INTERVAL_MS = 150;

void showMode3Default() {
  readAllSensors();
  lcdPrintLine(0, "MODE 3 - MANUAL");
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
  lastAdjustCwMs = millis() - ADJUST_HOLD_TIMEOUT_MS - 1;
  lastAdjustCcwMs = millis() - ADJUST_HOLD_TIMEOUT_MS - 1;
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
    if (step > 0 && selectedPocket < 4) {
      selectedPocket++;
      blinkSelOn = true;
      lastBlinkSelMs = now;
      drawSelectScreen();
    } else if (step < 0 && selectedPocket > 0) {
      selectedPocket--;
      blinkSelOn = true;
      lastBlinkSelMs = now;
      drawSelectScreen();
    }
    if (now - lastBlinkSelMs >= SELECT_BLINK_MS) {
      lastBlinkSelMs = now;
      blinkSelOn = !blinkSelOn;
      drawSelectScreen();
    }
  } else if (mode3State == M3_ADJUST) {
    // "계속 돌리는 동안만" 작동하는 홀드 방식 (파일럿2와 동일 UX)
    if (step > 0) lastAdjustCwMs = now;
    else if (step < 0) lastAdjustCcwMs = now;

    bool holdingCw = (now - lastAdjustCwMs) < ADJUST_HOLD_TIMEOUT_MS;
    bool holdingCcw = (now - lastAdjustCcwMs) < ADJUST_HOLD_TIMEOUT_MS;

    uint8_t groupIdx = POCKET_GROUP[selectedPocket];
    readAllSensors();

    bool shouldInflate = holdingCw && sensorRaw[selectedPocket] < pressureCeilingRaw;
    bool shouldDeflate = !shouldInflate && holdingCcw && sensorRaw[selectedPocket] > pressureFloorRaw;

    if (shouldInflate) {
      setValve3way(groupIdx, true);
      setValve2way(selectedPocket, true);
      runPump(groupIdx, pumpSpeed);
    } else if (shouldDeflate) {
      setValve3way(groupIdx, false);
      setValve2way(selectedPocket, true);
      stopPump(groupIdx);
    } else {
      setValve2way(selectedPocket, false);
      stopPump(groupIdx);
    }

    if (now - lastAdjustDrawMs >= ADJUST_DRAW_INTERVAL_MS) {
      lastAdjustDrawMs = now;
      drawAdjustScreen();
    }
  } else if (mode3State == M3_PRESET_SELECT) {
    if (step > 0 && presetSelectIdx < 2) {
      presetSelectIdx++;
      blinkSelOn = true;
      lastBlinkSelMs = now;
      drawPresetSelectScreen();
    } else if (step < 0 && presetSelectIdx > 0) {
      presetSelectIdx--;
      blinkSelOn = true;
      lastBlinkSelMs = now;
      drawPresetSelectScreen();
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
const char *MODE_NAMES[3] = {"AUTO FIT", "REALTIME", "MANUAL"};

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
  if (modeSelectIdx == 2) {
    showMode3Default();
  } else {
    showMode(modeSelectIdx);
  }
}

void enterModeSelect() {
  topState = TOP_MODE_SELECT;
  if (currentModeIdx >= 0 && currentModeIdx <= 2) modeSelectIdx = currentModeIdx;
  calState = CAL_IDLE;
  resetMode2();
  resetMode3();
  showModeSelectPreview();
}

void confirmModeSelect() {
  currentModeIdx = modeSelectIdx;
  topState = TOP_IN_MODE;
}

void tickModeSelect() {
  noInterrupts();
  int8_t step = rotAccum;
  rotAccum = 0;
  interrupts();

  bool changed = false;
  if (step > 0 && modeSelectIdx < 2) {
    modeSelectIdx++;
    changed = true;
  } else if (step < 0 && modeSelectIdx > 0) {
    modeSelectIdx--;
    changed = true;
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
  } else if (currentModeIdx == 1 && mode2State == M2_IDLE) {
    enterMode2Balanced();
  } else if (currentModeIdx == 2) {
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
    if (calState == CAL_IDLE) {
      enterModeSelect();
    } else {
      calState = CAL_IDLE;
      stopEverythingSafely();
      buzzerActive = false;
      showMode(0);
    }
  } else if (currentModeIdx == 1) {
    if (mode2State == M2_IDLE) {
      enterModeSelect();
    } else {
      resetMode2();
      showMode(1);
    }
  } else if (currentModeIdx == 2) {
    if (mode3State == M3_IDLE) {
      enterModeSelect();
    } else if (mode3State == M3_SELECT) {
      resetMode3();
      showMode3Default();
    } else if (mode3State == M3_ADJUST || mode3State == M3_PRESET_SELECT || mode3State == M3_PRESET_SAVED) {
      mode3State = M3_SELECT;
      blinkSelOn = true;
      lastBlinkSelMs = millis();
      drawSelectScreen();
    }
  }
}

void tickClickResolution() {
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
      if (clickPending && (now - clickPendingMs) <= DOUBLE_CLICK_WINDOW_MS) {
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
    presetDisplayUntilMs = millis() + PRESET_DISPLAY_HOLD_MS;
    Serial.print(F("프리셋 "));
    Serial.print(idx + 1);
    Serial.println(F(" 저장된 값 없음"));
    return;
  }

  lcdPrintLine(1, "Applying...");
  presetApplying = true;
  presetApplyingIdx = idx;
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

void handlePresetSwitches() {
  if (millis() - lastSwPollMs < SW_DEBOUNCE_MS) return;
  lastSwPollMs = millis();
  for (uint8_t i = 0; i < 3; i++) {
    bool pressed = readSwitch(i);
    if (pressed && !swWasPressed[i]) {
      if (calState == CAL_ANALYZING || presetApplying) {
        Serial.println(F("지금은 다른 하드웨어 동작 중이라 프리셋 적용 불가"));
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
  } else if (cmd == "STATUS") {
    Serial.print(F("CEIL,"));
    Serial.print(pressureCeilingRaw);
    Serial.print(F(",FLOOR,"));
    Serial.print(pressureFloorRaw);
    Serial.print(F(",TOL,"));
    Serial.print(toleranceRaw);
    Serial.print(F(",SPEED,"));
    Serial.println(pumpSpeed);
  }
}

// ================= 디버그 시리얼 =================
void printDebug() {
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

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, FALLING);

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

  handlePowerSwitch();
  tickClickResolution();

  if (systemOn) {
    handlePresetSwitches();

    if (presetApplying) {
      if (tickTargetSeek()) {
        presetApplying = false;
        stopAllPumps();
        closeAllValves();
        char line1[17];
        snprintf(line1, sizeof(line1), "%d %d %d %d",
                 presets[presetApplyingIdx][0], presets[presetApplyingIdx][1],
                 presets[presetApplyingIdx][2], presets[presetApplyingIdx][3]);
        lcdPrintLine(1, line1);
        presetDisplayUntilMs = millis() + PRESET_DISPLAY_HOLD_MS;
        Serial.println(F("프리셋 적용 완료"));
      }
    } else if (millis() < presetDisplayUntilMs) {
      // 프리셋 화면 유지 중 - 다른 화면 갱신 보류
    } else if (topState == TOP_MODE_SELECT) {
      tickModeSelect();
    } else {
      tickCalibration();
      tickMode2();
      tickMode3();
    }

    printDebug();
  }
}
