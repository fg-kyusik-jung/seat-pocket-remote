// 파일럿 3: 능동형 경도 조절 모드 검증
// 4포켓/2모터 통합 하드웨어 기준. 밸브(6채널)/LED(3)/스위치(3)는 PCF8574 2개로 확장,
// LCD는 PCF8574 인터페이스 내장형 I2C 1602. 전부 같은 I2C 버스(A4/A5)에 병렬 연결.
//
// ⚠ 아직 실측 안 된 가정들 (배선 후 반드시 검증):
//   - VALVE_ACTIVE_HIGH: 밸브 릴레이 익스팬더 출력 극성 (에너자이즈 시 HIGH인지 LOW인지)
//   - LED_ACTIVE_LOW: LED 익스팬더 출력 극성
//   - PUMP_FIXED_DIR: 파일럿1/2 실측치를 그대로 가정 (모터1/2 둘 다 동일 극성 배선이라는 가정, 개별 확인 필요)
//   - LCD_PCF8574 핀 배치(RS/RW/EN/BL/D4~D7 = P0~P7): 시중 1602 I2C 백팩의 통용 배치를 가정

#include <Wire.h>

// ---------- I2C 주소 ----------
const uint8_t ADDR_LCD = 0x27;
const uint8_t ADDR_IO = 0x20;    // LED 3 + 스위치 3
const uint8_t ADDR_VALVE = 0x24; // 밸브 릴레이 6채널 (2026-07-21 실측 확정: 점퍼 조정 결과 0x21이 아니라 0x24로 잡힘)

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
const uint8_t BIT_LED[3] = {0, 1, 2};   // 0=능동형, 1=실시간, 2=매뉴얼
const uint8_t BIT_SW[3] = {3, 4, 5};    // 0=능동형, 1=실시간, 2=매뉴얼

// ---------- 극성 가정 (실측 후 뒤집을 수 있음) ----------
const bool VALVE_ACTIVE_HIGH = true; // true면 1=에너자이즈(2way 열림/3way 펌프모드)
const bool LED_ACTIVE_LOW = false;    // true면 0=LED켜짐 (2026-07-21 실측: active-high로 확정)
const uint8_t PUMP_FIXED_DIR = 0;

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
const unsigned long DEBUG_PRINT_INTERVAL_MS = 200;

// ---------- 시스템/모드 상태 ----------
bool systemOn = false;
enum Mode { MODE_NONE, MODE_ACTIVE, MODE_REALTIME, MODE_MANUAL };
Mode currentMode = MODE_NONE;

enum ActiveState { AS_INFLATE, AS_SWEEP, AS_RETURN, AS_LOCKED };
ActiveState activeState = AS_INFLATE;

// SWEEP에서 찾은 최적점
bool bestFound = false;
uint16_t bestAvg = 0xFFFF;
uint16_t bestTarget[4] = {0, 0, 0, 0};

// RETURN 단계 그룹별 서브상태
enum ReturnPhase { RP_INFLATE, RP_DEFLATE, RP_DONE };
ReturnPhase returnPhase[2] = {RP_DONE, RP_DONE};

unsigned long stateEnteredMs = 0;

// ---------- 엔코더 (전원 ON/OFF 전용) ----------
volatile unsigned long lastEncoderInterruptMicros = 0;
const unsigned long ENCODER_DEBOUNCE_US = 5000;
bool swPrevState = HIGH;
unsigned long swPressStartMs = 0;
bool longPressFired = false;

// ---------- IO/밸브 익스팬더 섀도 레지스터 ----------
uint8_t ioShadow = 0xFF;    // 기본값: LED 전부 꺼짐(active-low 가정), 스위치 입력용 풀업
uint8_t valveShadow = 0x00; // 기본값: 전부 비에너자이즈(안전)

String inputLine;
unsigned long lastDebugPrintMs = 0;
unsigned long lastSweepTickMs = 0;

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

// ================= LCD (1602, PCF8574 인터페이스, 4비트 모드) =================
// 통용 배치: P0=RS, P1=RW, P2=EN, P3=Backlight, P4~P7=D4~D7
const uint8_t LCD_BIT_RS = 0;
const uint8_t LCD_BIT_RW = 1;
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
  lcdWrite4(0x2, false); // 4비트 모드 진입
  lcdCommand(0x28); // 4비트, 2행, 5x8 폰트
  lcdCommand(0x0C); // 디스플레이 on, 커서 off
  lcdCommand(0x06); // 커서 자동 우측 이동
  lcdCommand(0x01); // 클리어
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

// ================= IO 익스팬더 (LED + 스위치) =================
void setLed(uint8_t idx, bool on) {
  bool bitHigh = LED_ACTIVE_LOW ? !on : on;
  if (bitHigh) ioShadow |= (1 << BIT_LED[idx]);
  else ioShadow &= ~(1 << BIT_LED[idx]);
  pcfWrite(ADDR_IO, ioShadow);
}

void setLedExclusive(uint8_t idx) {
  for (uint8_t i = 0; i < 3; i++) setLed(i, i == idx);
}

bool readSwitch(uint8_t idx) {
  // 스위치 비트는 항상 1(풀업)로 유지해야 하므로 쓰기 전 보장
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

// ================= 부저 =================
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

// ================= 능동형 모드 상태머신 =================
void enterInflate() {
  activeState = AS_INFLATE;
  stateEnteredMs = millis();
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

  if (allDone || (millis() - stateEnteredMs) > INFLATE_TIMEOUT_MS) {
    stopAllPumps();
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    activeState = AS_SWEEP;
    stateEnteredMs = millis();
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
  bool timedOut = (millis() - stateEnteredMs) > SWEEP_TIMEOUT_MS;
  if (atFloor || timedOut) {
    for (uint8_t i = 0; i < 4; i++) setValve2way(i, false);
    if (!bestFound) {
      bestFound = true;
      for (uint8_t i = 0; i < 4; i++) bestTarget[i] = sensorRaw[i];
    }
    activeState = AS_RETURN;
    stateEnteredMs = millis();
    returnPhase[0] = RP_INFLATE;
    returnPhase[1] = RP_INFLATE;
  }
}

void tickReturnGroup(uint8_t groupIdx) {
  if (returnPhase[groupIdx] == RP_DONE) return;

  readAllSensors();
  uint8_t p0 = groupIdx == 0 ? 0 : 2;
  uint8_t p1 = groupIdx == 0 ? 1 : 3;

  if (returnPhase[groupIdx] == RP_INFLATE) {
    bool need0 = sensorRaw[p0] + toleranceRaw < bestTarget[p0];
    bool need1 = sensorRaw[p1] + toleranceRaw < bestTarget[p1];
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
    bool need0 = sensorRaw[p0] > bestTarget[p0] + toleranceRaw;
    bool need1 = sensorRaw[p1] > bestTarget[p1] + toleranceRaw;
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

void tickReturn() {
  tickReturnGroup(0);
  tickReturnGroup(1);
  if (returnPhase[0] == RP_DONE && returnPhase[1] == RP_DONE) {
    stopAllPumps();
    closeAllValves();
    activeState = AS_LOCKED;
    stateEnteredMs = millis();
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

// ================= 모드 전환 =================
void stopEverythingSafely() {
  stopAllPumps();
  closeAllValves();
  noTone(BUZZER_PIN);
  buzzerActive = false;
}

void switchMode(Mode m) {
  if (m == currentMode) return;
  stopEverythingSafely();
  currentMode = m;
  setLedExclusive((uint8_t)m - 1);
  if (m == MODE_ACTIVE) {
    enterInflate();
  }
}

// ================= 엔코더 (전원 ON/OFF) =================
void encoderSwISR_placeholder() {} // 회전(CLK/DT)은 이번 파일럿에서 미사용, 매뉴얼 모드용으로 예약

void handlePowerSwitch() {
  bool swNow = digitalRead(ENC_SW_PIN);
  unsigned long now = millis();

  // 누르고 있는 중 1초 경과 시점에 즉시 반응 (떼는 걸 기다리지 않음)
  if (swNow == LOW && swPrevState == LOW && !longPressFired &&
      (now - swPressStartMs) >= LONG_PRESS_MS) {
    longPressFired = true;
    systemOn = !systemOn;
    if (!systemOn) {
      stopEverythingSafely();
      currentMode = MODE_NONE;
      setLedExclusive(0xFF);
      lcdClear();
      lcdPrintLine(0, "SYSTEM OFF");
    } else {
      switchMode(MODE_ACTIVE); // 최초 진입 시 바로 능동형(1번) 모드로
    }
  }

  if (swNow == LOW && swPrevState == HIGH) {
    swPressStartMs = now;
    longPressFired = false;
  }
  swPrevState = swNow;
}

// ================= 모드 선택 스위치 =================
unsigned long lastSwPollMs = 0;
bool swWasPressed[3] = {false, false, false};

void handleModeSwitches() {
  if (millis() - lastSwPollMs < SW_DEBOUNCE_MS) return;
  lastSwPollMs = millis();
  for (uint8_t i = 0; i < 3; i++) {
    bool pressed = readSwitch(i);
    if (pressed && !swWasPressed[i]) {
      switchMode((Mode)(i + 1));
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
    Serial.println(toleranceRaw);
  }
}

// ================= LCD 상태 표시 =================
const char *activeStateName(ActiveState s) {
  switch (s) {
    case AS_INFLATE: return "ACTIVE:INFLATE";
    case AS_SWEEP: return "ACTIVE:SWEEP";
    case AS_RETURN: return "ACTIVE:RETURN";
    case AS_LOCKED: return "ACTIVE:LOCKED";
  }
  return "ACTIVE:?";
}

void updateLcd() {
  static char prevLine0[17] = "";
  static char prevLine1[17] = "";
  char line0[17];
  char line1[17];

  if (!systemOn) return;

  if (currentMode == MODE_NONE) {
    snprintf(line0, sizeof(line0), "MODE SELECT");
    snprintf(line1, sizeof(line1), "SW1/2/3=A/R/M");
  } else if (currentMode == MODE_ACTIVE) {
    snprintf(line0, sizeof(line0), "%s", activeStateName(activeState));
    snprintf(line1, sizeof(line1), "%d %d %d %d",
             rawToLevel(sensorRaw[0]), rawToLevel(sensorRaw[1]),
             rawToLevel(sensorRaw[2]), rawToLevel(sensorRaw[3]));
  } else if (currentMode == MODE_REALTIME) {
    snprintf(line0, sizeof(line0), "REALTIME");
    snprintf(line1, sizeof(line1), "(not impl.)");
  } else {
    snprintf(line0, sizeof(line0), "MANUAL");
    snprintf(line1, sizeof(line1), "(not impl.)");
  }

  if (strcmp(line0, prevLine0) != 0) {
    lcdPrintLine(0, line0);
    strncpy(prevLine0, line0, 17);
  }
  if (strcmp(line1, prevLine1) != 0) {
    lcdPrintLine(1, line1);
    strncpy(prevLine1, line1, 17);
  }
}

// ================= 디버그 시리얼 =================
void printDebug() {
  if (millis() - lastDebugPrintMs < DEBUG_PRINT_INTERVAL_MS) return;
  lastDebugPrintMs = millis();
  Serial.print(F("MODE,"));
  Serial.print((int)currentMode);
  Serial.print(F(",ASTATE,"));
  Serial.print((int)activeState);
  Serial.print(F(",RAW,"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(sensorRaw[i]);
    Serial.print(i < 3 ? "," : "");
  }
  Serial.print(F(",BESTAVG,"));
  Serial.println(bestFound ? bestAvg : 0);
}

// ================= setup/loop =================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);

  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  stopAllPumps();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pcfWrite(ADDR_VALVE, valveShadow); // 전 채널 비에너자이즈
  pcfWrite(ADDR_IO, ioShadow);       // LED 꺼짐 + 스위치 풀업

  lcdInit();
  lcdPrintLine(0, "SYSTEM OFF");
  lcdPrintLine(1, "LONG PRESS = ON");

  inputLine.reserve(64);
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

  if (systemOn) {
    handleModeSwitches();
    if (currentMode == MODE_ACTIVE) {
      runActiveMode();
      bool alarm = (activeState == AS_INFLATE) &&
                    (sensorRaw[0] >= pressureCeilingRaw || sensorRaw[1] >= pressureCeilingRaw ||
                     sensorRaw[2] >= pressureCeilingRaw || sensorRaw[3] >= pressureCeilingRaw);
      updateBuzzer(alarm);
    } else {
      readAllSensors();
    }
    updateLcd();
    printDebug();
  }
}
