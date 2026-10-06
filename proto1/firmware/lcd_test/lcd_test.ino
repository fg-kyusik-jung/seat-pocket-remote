// LCD 1602(PCF8574 내장, I2C) 표시 확인용 테스트 스케치.
// pilot3.ino와 동일한 저수준 LCD 드라이버(4비트 모드, P0~P7=RS/RW/EN/BL/D4~D7 가정)를 그대로 사용.
// 2초마다 화면을 바꿔가며 실제 능동형 모드에서 쓸 표시 형식(상태명 + 포켓 레벨/raw값)을 미리 확인.

#include <Wire.h>

const uint8_t ADDR_LCD = 0x27;

const uint8_t LCD_BIT_RS = 0;
const uint8_t LCD_BIT_RW = 1;
const uint8_t LCD_BIT_EN = 2;
const uint8_t LCD_BIT_BL = 3;

void pcfWrite(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

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

// ---- 테스트 화면들 ----
const char *screens[][2] = {
  {"PILOT3 LCD TEST", "ADDR 0x27 OK"},
  {"ACTIVE:INFLATE", "7 6 8 5"},
  {"ACTIVE:SWEEP", "5 5 6 5"},
  {"ACTIVE:LOCKED", "5 5 5 5"},
  {"RAW VALUES", "312 288 401 350"},
  {"HELLO WORLD 1234", "abcdefghijklmnop"},
};
const uint8_t NUM_SCREENS = sizeof(screens) / sizeof(screens[0]);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcdInit();
  Serial.println(F("LCD 테스트 시작 - 2초마다 화면 전환"));
}

void loop() {
  for (uint8_t i = 0; i < NUM_SCREENS; i++) {
    lcdPrintLine(0, screens[i][0]);
    lcdPrintLine(1, screens[i][1]);
    Serial.print(F("화면 "));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(screens[i][0]);
    Serial.print(F(" / "));
    Serial.println(screens[i][1]);
    delay(2000);
  }
}
