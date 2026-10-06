// 리모컨측 (XIAO ESP32-C3): 엔코더 + LCD(1602,I2C 0x27) + PCF8574(0x24,LED3+스위치3) + 부저
// 좌판측 브리지-ESP32와 ESP-NOW로 무선 통신. 통신 프로토콜은 pilot3_rev1_remote.ino 상단 주석 참고.
//
// ⚠ 배선/실측 전 확인 필요:
//   - BRIDGE_MAC: 실제 브리지-ESP32의 MAC 주소로 교체 필요 (브리지 부팅 시 시리얼에 출력됨)
//   - LED_ACTIVE_LOW: 기존 좌판 PCF8574 기준값(false=HIGH가 켜짐)을 그대로 가정, 이 보드는 새 PCF8574라 재확인 필요
//   - 지금은 브리지 1개만 대상(TARGET_MAC 고정) — 여러 좌판 시스템 전환은 추후 리스트+선택 UI로 확장 예정

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

// ESP-NOW 수신 콜백은 메인 루프와 다른 태스크(스레드)에서 실행됨 - 콜백 안에서 바로 I2C(LCD)를
// 건드리면 메인 루프가 동시에 I2C(PCF8574 스위치 읽기)를 쓸 때 버스가 충돌해서 LCD 글자가 깨짐.
// (2026-10-06 발견) 콜백에서는 큐에 메시지만 넣고, 실제 I2C 작업은 메인 루프에서만 순서대로 처리하도록 분리.
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
QueueHandle_t cmdQueue;
const uint8_t CMD_MAX_LEN = 64;

// ---------- 연결상태 하트비트 (2026-10-06 추가) ----------
// "통신이 끊겼는지 vs 하드웨어가 고장났는지" 구분이 안 되는 문제를 해결하기 위해,
// 무선 링크 자체의 생존여부를 별도로 확인하고 화면에 명시적으로 표시한다.
const unsigned long PING_INTERVAL_MS = 500;
const unsigned long LINK_TIMEOUT_MS = 1500; // 이 시간 동안 PONG 못 받으면 연결끊김으로 판단
unsigned long lastPingMs = 0;
volatile unsigned long lastPongMs = 0;
bool linkWasUp = false; // 직전 루프에서의 연결상태 (전환 시점 감지용)

// ---------- 대상 브리지 MAC (2026-10-01 esptool로 실측한 값, bridge_esp32 재플래싱 시 재확인 필요) ----------
uint8_t BRIDGE_MAC[6] = {0x14, 0x63, 0x93, 0xBD, 0xC4, 0x68};

// ---------- 핀 ----------
const uint8_t ENC_CLK_PIN = D0;
const uint8_t ENC_DT_PIN = D1;
const uint8_t ENC_SW_PIN = D2;
const uint8_t BUZZER_PIN = D3;
// D4(SDA)/D5(SCL) = I2C 기본핀

// ---------- I2C 주소 ----------
const uint8_t ADDR_LCD = 0x27;
const uint8_t ADDR_IO = 0x38; // LED3(P0~P2) + 스위치3(P3~P5) - PCF8574A, A0/A1/A2 전부 GND로 고정(2026-10-06)

const bool LED_ACTIVE_LOW = false; // 실측 필요 (좌판 기존값 가정)

// ================= ESP-NOW 송신 =================
// ESP-NOW는 전달을 보장하지 않아서(2026-10-06 실측으로 확인 - LCD/백라이트 명령이 간헐적으로
// 유실되어 화면이 따로 노는 증상 발생), 전송 콜백으로 실제 도달 여부를 확인하고 실패 시 재전송한다.
volatile bool sendCbDone = false;
volatile bool sendCbSuccess = false;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  sendCbSuccess = (status == ESP_NOW_SEND_SUCCESS);
  sendCbDone = true;
}

bool sendToUno(const String &msg) {
  const uint8_t MAX_RETRY = 4;
  for (uint8_t attempt = 0; attempt < MAX_RETRY; attempt++) {
    sendCbDone = false;
    sendCbSuccess = false;
    esp_err_t r = esp_now_send(BRIDGE_MAC, (const uint8_t *)msg.c_str(), msg.length());
    if (r != ESP_OK) continue; // 큐에 넣는 것 자체가 실패하면 바로 재시도

    unsigned long start = millis();
    while (!sendCbDone && (millis() - start) < 20) { delay(1); }

    if (sendCbDone && sendCbSuccess) {
      if (attempt > 0) {
        Serial.print("[TX] ");
        Serial.print(msg);
        Serial.print(" (재시도 ");
        Serial.print(attempt);
        Serial.println("회 후 성공)");
      }
      return true;
    }
  }
  Serial.print("[TX] ");
  Serial.print(msg);
  Serial.println(" (전송 최종 실패)");
  return false;
}

// ================= 엔코더 (풀 쿼드러처 디코드, pilot3와 동일 알고리즘) =================
volatile uint8_t encRawState = 0;
volatile int8_t encQuarterAccum = 0;
volatile int8_t rotAccum = 0;
const int8_t ENC_QUAD_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0,
};

void IRAM_ATTR encoderISR() {
  uint8_t clk = digitalRead(ENC_CLK_PIN);
  uint8_t dt = digitalRead(ENC_DT_PIN);
  uint8_t state = (clk << 1) | dt;
  uint8_t idx = (encRawState << 2) | state;
  encRawState = state;
  encQuarterAccum += ENC_QUAD_TABLE[idx & 0x0F];
  if (encQuarterAccum >= 4) {
    rotAccum++;
    encQuarterAccum = 0;
  } else if (encQuarterAccum <= -4) {
    rotAccum--;
    encQuarterAccum = 0;
  }
  if (state == 3) encQuarterAccum = 0;
}

bool lastEncSw = HIGH;
bool lastPresetSw[3] = {HIGH, HIGH, HIGH};
unsigned long lastResyncMs = 0; // SW/PSW 상태를 주기적으로 재전송(패킷 유실 자동복구용)

// 디바운스용 "날것(raw)" 상태 추적 (스위치 채터링으로 인한 오동작 방지, 2026-10-06 발견)
bool lastRawEncSw = HIGH;
unsigned long lastRawEncSwChangeMs = 0;
bool lastRawPresetSw[3] = {HIGH, HIGH, HIGH};
unsigned long lastRawPresetSwChangeMs[3] = {0, 0, 0};

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

// ================= LED + 프리셋 스위치 (PCF8574 0x24) =================
uint8_t ioShadow = 0b00111000; // LED 3개 OFF, 스위치 3개 풀업 HIGH

void setLed(uint8_t idx, bool on) {
  bool bitHigh = LED_ACTIVE_LOW ? !on : on;
  if (bitHigh) ioShadow |= (1 << idx);
  else ioShadow &= ~(1 << idx);
  pcfWrite(ADDR_IO, ioShadow);
}

bool readPresetSwitch(uint8_t idx) {
  uint8_t raw = pcfRead(ADDR_IO);
  return (raw & (1 << (3 + idx))) == 0; // LOW = 눌림
}

// ================= LCD (1602, PCF8574 내장, 4비트 모드) =================
const uint8_t LCD_BIT_RS = 0;
const uint8_t LCD_BIT_EN = 2;
const uint8_t LCD_BIT_BL = 3;
bool lcdBacklightState = true;

void lcdPulseEnable(uint8_t data) {
  pcfWrite(ADDR_LCD, data | (1 << LCD_BIT_EN));
  delayMicroseconds(1);
  pcfWrite(ADDR_LCD, data & ~(1 << LCD_BIT_EN));
  delayMicroseconds(50);
}

void lcdWrite4(uint8_t nibble, bool rs) {
  uint8_t data = lcdBacklightState ? (1 << LCD_BIT_BL) : 0;
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
  lcdBacklightState = false;
  pcfWrite(ADDR_LCD, 0x00);
}

void lcdBacklightOn() {
  lcdBacklightState = true;
  pcfWrite(ADDR_LCD, (1 << LCD_BIT_BL));
}

void lcdCreateChar(uint8_t location, const uint8_t *charmap) {
  lcdCommand(0x40 | ((location & 0x7) << 3));
  for (uint8_t i = 0; i < 8; i++) lcdChar(charmap[i]);
}

// ================= 부저 =================
void remoteTone(uint16_t freq) {
  tone(BUZZER_PIN, freq);
}

void remoteNoTone() {
  noTone(BUZZER_PIN);
}

// ================= 우노->리모컨 명령 처리 (ESP-NOW 수신) =================
void handleUnoCommand(const String &line) {
  int c1 = line.indexOf(',');
  String cmd = (c1 < 0) ? line : line.substring(0, c1);

  if (cmd == "LCD") {
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 > 0) {
      uint8_t row = (uint8_t)line.substring(c1 + 1, c2).toInt();
      String text = line.substring(c2 + 1);
      lcdPrintLine(row, text.c_str());
    }
  } else if (cmd == "LCDCLR") {
    lcdClear();
  } else if (cmd == "LCDCC") {
    // LCDCC,<loc>,<b0>,<b1>,...,<b7>
    int pos = c1 + 1;
    int nextC = line.indexOf(',', pos);
    uint8_t loc = (uint8_t)line.substring(pos, nextC).toInt();
    uint8_t bytes[8];
    pos = nextC + 1;
    for (uint8_t i = 0; i < 8; i++) {
      nextC = line.indexOf(',', pos);
      String tok = (nextC > 0) ? line.substring(pos, nextC) : line.substring(pos);
      bytes[i] = (uint8_t)tok.toInt();
      pos = nextC + 1;
    }
    lcdCreateChar(loc, bytes);
  } else if (cmd == "BL") {
    if (line.substring(c1 + 1).toInt() != 0) lcdBacklightOn();
    else lcdBacklightOff();
  } else if (cmd == "LED") {
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 > 0) {
      uint8_t idx = (uint8_t)line.substring(c1 + 1, c2).toInt();
      bool on = line.substring(c2 + 1).toInt() != 0;
      if (idx < 3) setLed(idx, on);
    }
  } else if (cmd == "BUZ") {
    remoteTone((uint16_t)line.substring(c1 + 1).toInt());
  } else if (cmd == "BUZOFF") {
    remoteNoTone();
  }
}

void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // PONG(하트비트 응답)은 연결상태 타임스탬프만 갱신하고 끝 - handleUnoCommand로 안 넘김
  if (len == 4 && memcmp(data, "PONG", 4) == 0) {
    lastPongMs = millis();
    Serial.print("[PONG] 수신함, lastPongMs=");
    Serial.println(lastPongMs);
    return;
  }

  Serial.print("[RX] from ");
  for (int i = 0; i < 6; i++) { Serial.printf("%02X", info->src_addr[i]); if (i < 5) Serial.print(":"); }
  Serial.print(" len=");
  Serial.print(len);
  Serial.print(" data=");
  for (int i = 0; i < len; i++) Serial.print((char)data[i]);
  Serial.println();

  // ESP-NOW는 전송마다 패킷 단위로 구분되어 도착하므로(UART처럼 줄바꿈으로
  // 메시지 경계를 나눌 필요가 없음), 패킷 하나를 그대로 명령 1개로 처리한다.
  // 단, 여기(콜백/다른 태스크)에서 바로 처리하지 않고 큐에만 넣는다 - 실제 처리(LCD I2C 등)는
  // 메인 루프에서만 하도록 분리해서 I2C 버스 동시접근 충돌(글자 깨짐)을 방지.
  char buf[CMD_MAX_LEN];
  int copyLen = len < (CMD_MAX_LEN - 1) ? len : (CMD_MAX_LEN - 1);
  memcpy(buf, data, copyLen);
  buf[copyLen] = '\0';
  xQueueSend(cmdQueue, buf, 0);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("---- I2C 스캔 시작 ----");
  uint8_t foundCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("발견: 0x");
      Serial.println(addr, HEX);
      foundCount++;
    }
  }
  if (foundCount == 0) Serial.println("장치 없음 - 배선(SDA/SCL/VCC/GND) 확인할 것");
  Serial.print("총 ");
  Serial.print(foundCount);
  Serial.println("개 발견");

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  encRawState = (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), encoderISR, CHANGE);

  pinMode(BUZZER_PIN, OUTPUT);
  remoteNoTone();

  pcfWrite(ADDR_IO, ioShadow);
  lcdInit();
  lcdClear();

  cmdQueue = xQueueCreate(16, CMD_MAX_LEN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // WiFi 절전모드가 ESP-NOW 수신을 간헐적으로 지연/누락시키는 문제 방지
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BRIDGE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&peer);
  Serial.print("peer 등록 결과: ");
  Serial.println(peerResult == ESP_OK ? "성공" : "실패");

  Serial.print("리모컨 시작. 내 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("대상 브리지 MAC: ");
  for (int i = 0; i < 6; i++) { Serial.printf("%02X", BRIDGE_MAC[i]); if (i < 5) Serial.print(":"); }
  Serial.println();
}

void loop() {
  // ---- 연결상태 하트비트 (가장 먼저 처리 — 유선처럼 "항상 이어진" 상태를 흉내내기 위한 토대) ----
  if (millis() - lastPingMs >= PING_INTERVAL_MS) {
    lastPingMs = millis();
    esp_now_send(BRIDGE_MAC, (const uint8_t *)"PING", 4);
  }

  bool linkUp = (millis() - lastPongMs) < LINK_TIMEOUT_MS;
  if (linkUp != linkWasUp) {
    linkWasUp = linkUp;
    Serial.print("[LINK] 상태전환 -> ");
    Serial.println(linkUp ? "UP" : "DOWN");
    if (!linkUp) {
      lcdBacklightOn();
      lcdClear();
      lcdPrintLine(0, "CONNECTING...");
      lcdPrintLine(1, "(wireless link)");
    }
  }

  if (!linkUp) {
    // 연결 끊긴 동안은 우노 명령 처리/엔코더·스위치 전송 등 정상 UI 로직을 전부 보류하고
    // 화면에 CONNECTING 표시만 유지 - "통신 문제"와 "하드웨어 고장"을 명확히 구분하기 위함.
    delay(5);
    return;
  }

  // ESP-NOW로 받아서 큐에 쌓인 명령을 메인 루프에서만 순서대로 처리 (I2C는 여기서만 건드림)
  char buf[CMD_MAX_LEN];
  while (xQueueReceive(cmdQueue, buf, 0) == pdTRUE) {
    String line = String(buf);
    line.trim();
    if (line.length() > 0) handleUnoCommand(line);
  }

  // 엔코더 회전 -> ENC,<delta>
  if (rotAccum != 0) {
    noInterrupts();
    int8_t delta = rotAccum;
    rotAccum = 0;
    interrupts();
    sendToUno("ENC," + String(delta));
  }

  // 엔코더 스위치 레벨 -> SW,<0|1> (디바운스: 값이 DEBOUNCE_MS 이상 안정적으로 유지될 때만 반영)
  // ESP-NOW는 전달 보장이 없어서(패킷 하나 유실되면 우노 쪽 상태가 영구적으로 꼬임),
  // 상태 바뀔 때만 보내지 않고 일정 주기로 "현재 상태"를 계속 반복 전송해서 자동 복구되게 함.
  const unsigned long DEBOUNCE_MS = 30;
  bool rawSw = digitalRead(ENC_SW_PIN);
  if (rawSw != lastRawEncSw) {
    lastRawEncSw = rawSw;
    lastRawEncSwChangeMs = millis();
  }
  bool sw = lastEncSw;
  if (rawSw != lastEncSw && (millis() - lastRawEncSwChangeMs) >= DEBOUNCE_MS) {
    sw = rawSw;
  }
  bool swChanged = (sw != lastEncSw);
  lastEncSw = sw;

  // 프리셋 스위치 3개 레벨 -> PSW,<idx>,<0|1> (동일하게 디바운스 적용)
  bool pswLevel[3];
  bool pswChanged[3];
  for (uint8_t i = 0; i < 3; i++) {
    bool rawPressed = readPresetSwitch(i);
    bool rawLevel = rawPressed ? LOW : HIGH;
    if (rawLevel != lastRawPresetSw[i]) {
      lastRawPresetSw[i] = rawLevel;
      lastRawPresetSwChangeMs[i] = millis();
    }
    pswLevel[i] = lastPresetSw[i];
    if (rawLevel != lastPresetSw[i] && (millis() - lastRawPresetSwChangeMs[i]) >= DEBOUNCE_MS) {
      pswLevel[i] = rawLevel;
    }
    pswChanged[i] = (pswLevel[i] != lastPresetSw[i]);
    lastPresetSw[i] = pswLevel[i];
  }

  const unsigned long RESYNC_INTERVAL_MS = 300;
  bool dueForResync = (millis() - lastResyncMs) >= RESYNC_INTERVAL_MS;
  if (dueForResync) lastResyncMs = millis();

  if (swChanged || dueForResync) {
    sendToUno("SW," + String(sw == HIGH ? 1 : 0));
  }
  for (uint8_t i = 0; i < 3; i++) {
    if (pswChanged[i] || dueForResync) {
      sendToUno("PSW," + String(i) + "," + String(pswLevel[i] == HIGH ? 1 : 0));
    }
  }

  delay(5); // 스위치 폴링 간격 (엔코더 자체는 인터럽트라 영향 없음)
}
