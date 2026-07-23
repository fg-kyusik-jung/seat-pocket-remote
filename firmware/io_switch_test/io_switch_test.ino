// IO 익스팬더(PCF8574, LED3+스위치3) + 엔코더 전원 ON/OFF만 떼어서 테스트.
// pilot3.ino의 해당 로직과 동일 (밸브/펌프/센서 없음). 2way 밸브 도착 전 미리 검증용.

#include <Wire.h>
#include <EEPROM.h>

const uint8_t ADDR_LCD = 0x27;
const uint8_t ADDR_IO = 0x24; // 테스트: 원래 0x20 모듈 대신 0x24(밸브용) 모듈로 시험

const uint8_t ENC_CLK_PIN = 2;
const uint8_t ENC_DT_PIN = 3;
const uint8_t ENC_SW_PIN = 4;
const uint8_t BUZZER_PIN = 7;

const uint8_t BIT_LED[3] = {0, 1, 2};
const uint8_t BIT_SW[3] = {3, 4, 5};
const bool LED_ACTIVE_LOW = false; // 2026-07-21 실측 확정치로 원복 (active-low 재배선 안 하기로 함)

const unsigned long LONG_PRESS_MS = 1000;
const unsigned long SW_DEBOUNCE_MS = 150;

uint8_t ioShadow = 0xFF;
bool systemOn = false;

bool encSwPrevState = HIGH;
unsigned long encSwPressStartMs = 0;
bool longPressFired = false;

// 싱글/더블클릭 판별용 (짧은 클릭은 즉시 실행하지 않고 잠깐 대기해서 두 번째 클릭이 오는지 확인)
bool clickPending = false;
unsigned long clickPendingMs = 0;
const unsigned long DOUBLE_CLICK_WINDOW_MS = 400;

unsigned long lastSwPollMs = 0;
bool swWasPressed[3] = {false, false, false};
int8_t currentModeIdx = 0;

volatile int8_t rotAccum = 0;
volatile unsigned long lastEncIsrMicros = 0;
volatile uint16_t isrFireCount = 0;
const unsigned long ENC_ISR_DEBOUNCE_US = 5000;

void encoderISR() {
  isrFireCount++;
  unsigned long now = micros();
  if (now - lastEncIsrMicros < ENC_ISR_DEBOUNCE_US) return;
  lastEncIsrMicros = now;
  if (digitalRead(ENC_DT_PIN)) rotAccum--;
  else rotAccum++;
}

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
  return (raw & (1 << BIT_SW[idx])) == 0;
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

// 커스텀 문자 등록 (CGRAM 0~7). 0번은 C문자열 종료바이트와 겹치므로 피하고 1~7만 사용.
void lcdCreateChar(uint8_t location, const uint8_t *charmap) {
  lcdCommand(0x40 | ((location & 0x7) << 3));
  for (uint8_t i = 0; i < 8; i++) lcdChar(charmap[i]);
}

const uint8_t CHAR_BOOKMARK = 1;
const uint8_t BOOKMARK_GLYPH[8] = {0x1F, 0x11, 0x11, 0x11, 0x11, 0x15, 0x1B, 0x00};

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

// ================= 모드1(AUTO FIT) 캘리브레이션 =================
enum CalState { CAL_IDLE, CAL_ANALYZING, CAL_COMPLETE_SHOW, CAL_LOCKED };
CalState calState = CAL_IDLE;
unsigned long calStateStartMs = 0;
unsigned long lastLetterMs = 0;
uint8_t revealedChars = 0;

const char LINE0_TEXT[] = "Analyzing your";
const char LINE1_TEXT[] = "body pressure";
const uint8_t LINE0_LEN = sizeof(LINE0_TEXT) - 1; // 14
const uint8_t LINE1_LEN = sizeof(LINE1_TEXT) - 1; // 13
const uint8_t ANALYZE_TOTAL_LEN = LINE0_LEN + LINE1_LEN;
const unsigned long LETTER_REVEAL_MS = 150;
// ---- 준비 필요: 지금은 실제 센서/펌프가 없어서 10초 경과로 "완료"를 임시 가정함.
// 실제 하드웨어 연결되면 압력 안정화 판정 조건으로 교체할 것.
const unsigned long CAL_DURATION_MS = 10000;

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

// 문구 다 뜬 뒤 "로딩 중" 표시용 점 애니메이션 (1~3개 반복, line1 뒤에 이어붙임)
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
  Serial.println(F("캘리브레이션 시작"));
}

void tickCalibration() {
  if (calState == CAL_IDLE || calState == CAL_LOCKED) return;

  unsigned long now = millis();
  unsigned long elapsed = now - calStateStartMs;

  if (calState == CAL_ANALYZING) {
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
    if (elapsed >= CAL_DURATION_MS) { // ---- 준비 필요: 실측 조건으로 교체
      calState = CAL_COMPLETE_SHOW;
      calStateStartMs = now;
      lcdPrintLine(0, "Analyze Complete");
      lcdPrintLine(1, "!");
      playCompleteChime();
      Serial.println(F("캘리브레이션 완료"));
    }
  } else if (calState == CAL_COMPLETE_SHOW) {
    if (elapsed >= 2000) {
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
const uint8_t ALERT_BEEP_MAX = 4; // 부저는 4번만 울리고 멈춤 (깜빡임/LCD는 계속 유지)

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
  showWellBalanced();
  // ---- 준비: 모드 진입 시점의 경도(밸브 잠금) 상태를 그대로 유지하는 로직 필요 (하드웨어 연결 후 구현)
  Serial.println(F("모드2 진입 - Well balanced"));
}

void enterMode2Alert() {
  mode2State = M2_ALERT;
  alertStartMs = millis();
  lastBlinkMs = 0;
  blinkOn = false;
  alertBeepCount = 0;
  showSitUpStraight();
  // ---- 준비: 원하는 자세로 유도하는 실제 액추에이션(밸브/펌프 조정) 로직 필요 (하드웨어 연결 후 구현)
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
      // ---- 준비: 모드 진입 시점의 경도 상태로 되돌리는 로직 필요 (하드웨어 연결 후 구현)
      noTone(BUZZER_PIN);
      lcdBacklightOn();
      playCompleteChime(); // 모드1과 동일한 도-미-솔-도 복귀 알림음
      enterMode2Balanced();
    }
  }
}

// ================= 모드3(MANUAL) 포켓 선택/조절 =================
enum Mode3State { M3_IDLE, M3_SELECT, M3_ADJUST, M3_PRESET_SELECT, M3_PRESET_SAVED };
Mode3State mode3State = M3_IDLE;
uint8_t selectedPocket = 0; // 0~3 = 포켓 1~4, 4 = 북마크(프리셋 저장)
uint8_t pocketLevels[4] = {3, 3, 5, 5}; // 임시값 (실제 센서 연결 전)
uint8_t presetSelectIdx = 0; // 0~2 = 프리셋 1~3
uint8_t presets[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // 프리셋별 저장된 포켓 레벨 (EEPROM과 동기화됨)
const int EEPROM_PRESET_BASE = 0; // 프리셋 3개 * 4바이트 = 12바이트(주소 0~11) 사용

void loadPresetsFromEeprom() {
  for (uint8_t p = 0; p < 3; p++) {
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t v = EEPROM.read(EEPROM_PRESET_BASE + p * 4 + i);
      presets[p][i] = (v >= 1 && v <= 10) ? v : 0; // 한 번도 저장 안 된 칸(0xFF 등)은 0으로 간주
    }
  }
}

void savePresetToEeprom(uint8_t idx) {
  for (uint8_t i = 0; i < 4; i++) {
    EEPROM.update(EEPROM_PRESET_BASE + idx * 4 + i, presets[idx][i]); // update()라 값이 같으면 실제로 안 씀(수명 보호)
  }
}

unsigned long lastBlinkSelMs = 0;
bool blinkSelOn = true;
const unsigned long SELECT_BLINK_MS = 500;

void showMode3Default() {
  lcdPrintLine(0, "MODE 3 - MANUAL");
  char line1[17];
  snprintf(line1, sizeof(line1), "%d %d %d %d",
           pocketLevels[0], pocketLevels[1], pocketLevels[2], pocketLevels[3]);
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
  snprintf(line1, sizeof(line1), "Level %d", pocketLevels[selectedPocket]);
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
  for (uint8_t i = 0; i < 4; i++) presets[idx][i] = pocketLevels[i]; // 기존 값 무조건 덮어씀
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
    if (step != 0) {
      int16_t newLevel = pocketLevels[selectedPocket] + step;
      if (newLevel < 1) newLevel = 1;
      if (newLevel > 10) newLevel = 10;
      pocketLevels[selectedPocket] = (uint8_t)newLevel;
      // ---- 준비: 실제 펌프/밸브로 포켓을 부풀리고/줄이는 로직 필요 (하드웨어 연결 후 구현)
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

  if (changed) {
    showModeSelectPreview();
  }
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
      // ---- 준비: 휘발성 배열이 아니라 실제 EEPROM에 저장하는 로직 필요 (전원 꺼지면 사라짐, 하드웨어 설계 확정 후 구현)
    }
    // M3_ADJUST, M3_PRESET_SAVED에서는 싱글클릭 동작 없음 (더블클릭=뒤로가기로 통일)
  }
}

void handleDoubleClick() {
  if (topState == TOP_MODE_SELECT) return; // 최상위에서는 더블클릭 할 게 없음

  if (currentModeIdx == 0) {
    if (calState == CAL_IDLE) {
      enterModeSelect();
    } else {
      calState = CAL_IDLE;
      noTone(BUZZER_PIN);
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

  // 누르고 있는 중 1초 경과 시점에 즉시 반응 (떼는 걸 기다리지 않음)
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
      noTone(BUZZER_PIN);
      calState = CAL_IDLE;
      resetMode2();
      resetMode3();
      clickPending = false;
      showSystemOff();
    }
  }

  if (swNow == LOW && encSwPrevState == HIGH) {
    encSwPressStartMs = now;
    longPressFired = false;
  } else if (swNow == HIGH && encSwPrevState == LOW) {
    // 롱프레스가 발동 안 한 채로 뗀 경우 = 짧게 누른 것 (싱글/더블클릭 판별은 잠깐 대기 후 처리)
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

unsigned long presetDisplayUntilMs = 0;
const unsigned long PRESET_DISPLAY_HOLD_MS = 1500; // 이 시간 동안은 다른 화면 갱신이 덮어쓰지 못하게 보류

void activatePreset(uint8_t idx) {
  presetDisplayUntilMs = millis() + PRESET_DISPLAY_HOLD_MS;
  setLedExclusive(idx);
  lcdBacklightOn();
  char line0[17];
  snprintf(line0, sizeof(line0), "PRESET #%d", idx + 1);
  lcdPrintLine(0, line0);
  // 저장된 값이 제대로 반영되는지 눈으로 확인하기 위해 실제 저장값을 표시
  char line1[17];
  snprintf(line1, sizeof(line1), "%d %d %d %d",
           presets[idx][0], presets[idx][1], presets[idx][2], presets[idx][3]);
  lcdPrintLine(1, line1);
  // ---- 준비: 이 저장값을 실제로 펌프/밸브에 적용하는 로직 필요 (하드웨어 연결 후 구현)
  Serial.print(F("프리셋 "));
  Serial.print(idx + 1);
  Serial.print(F(" 활성화: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(presets[idx][i]);
    Serial.print(' ');
  }
  Serial.println();
}

// 어느 모드/화면에 있든 눌리면 즉시 프리셋을 불러옴 (전역 단축키)
void handlePresetSwitches() {
  if (millis() - lastSwPollMs < SW_DEBOUNCE_MS) return;
  lastSwPollMs = millis();
  for (uint8_t i = 0; i < 3; i++) {
    bool pressed = readSwitch(i);
    if (pressed && !swWasPressed[i]) {
      activatePreset(i);
    }
    swWasPressed[i] = pressed;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  loadPresetsFromEeprom();

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, FALLING);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pcfWrite(ADDR_IO, ioShadow);

  lcdInit();
  lcdCreateChar(CHAR_BOOKMARK, BOOKMARK_GLYPH);
  lcdClear();
  lcdBacklightOff(); // 전원 OFF 상태 = LCD 꺼짐

  Serial.println(F("IO/엔코더 테스트 시작 - 엔코더 롱프레스로 전원 ON/OFF"));
}

unsigned long lastEncDebugMs = 0;

void printEncoderDebug() {
  if (millis() - lastEncDebugMs < 100) return;
  lastEncDebugMs = millis();
  Serial.print(F("CLK,"));
  Serial.print(digitalRead(ENC_CLK_PIN));
  Serial.print(F(",DT,"));
  Serial.print(digitalRead(ENC_DT_PIN));
  Serial.print(F(",ROTACCUM,"));
  Serial.print(rotAccum);
  Serial.print(F(",ISRFIRES,"));
  Serial.println(isrFireCount);
}

void loop() {
  handlePowerSwitch();
  tickClickResolution();
  printEncoderDebug();
  if (systemOn) {
    handlePresetSwitches();
    if (millis() < presetDisplayUntilMs) {
      // 프리셋 활성화 화면을 보여주는 중 - 다른 화면 갱신(깜빡임 등)이 덮어쓰지 않도록 대기
    } else if (topState == TOP_MODE_SELECT) {
      tickModeSelect();
    } else {
      tickCalibration();
      tickMode2();
      tickMode3();
    }
  }
}
