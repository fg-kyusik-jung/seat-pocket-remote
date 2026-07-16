// 좌판 경도 조절 시스템 - 실험용 펌웨어 (최소 구성: 1펌프 공용 + 섹터별 밸브/센서)
// 시리얼 프로토콜
//   Arduino -> PC : "T,<millis>,<sector0_raw>,<sector1_raw>,...\n"   (주기 전송, raw = analogRead 0~1023)
//                    "M,<pump_speed 0-255>,<pump_dir 0|1>\n"          (주기 전송, 가변저항으로 결정된 현재 펌프 속도)
//                    "READY,<NUM_SECTORS>\n"                          (부팅 시 / STATUS 요청 시)
//   PC -> Arduino : "VALVE,<sector_idx>,<0|1>\n"   0=닫힘(밸브 OFF, 펌프측과 연결) 1=열림(대기측 배기)
//                    "PUMP,<dir 0|1>\n"             방향만 소프트웨어로 지정, 속도는 가변저항이 결정
//                    "ALARM,<sector_idx>,<raw_threshold>\n"  섹터별 경보 raw 임계값 실시간 조정
//                    "STATUS\n"
//
// 섹터 확장 시 NUM_SECTORS와 VALVE_PINS/SENSOR_PINS/alarmRawThreshold 배열만 늘리면 됨.

const uint8_t NUM_SECTORS = 1;

const uint8_t VALVE_PINS[NUM_SECTORS] = {2};
const uint8_t SENSOR_PINS[NUM_SECTORS] = {A0};

const uint8_t PUMP_PIN_A = 9;
const uint8_t PUMP_PIN_B = 10;
const uint8_t POT_PIN = A1;
const uint16_t POT_DEADBAND = 10;  // 가변저항 하단 노이즈로 인한 미세 회전 오작동 방지

const uint8_t BUZZER_PIN = 4;
const unsigned int BUZZER_FREQ_HZ = 440;

// 목표압력 상한 실측 확정값: raw≈500 (40kPa까지 쓸 필요 없어 kPa 변환은 보류, 2026-07-13 확정).
// 필요시 ALARM 명령으로 섹터별 임계값 조정 가능.
uint16_t alarmRawThreshold[NUM_SECTORS] = {500};
bool buzzerActive = false;

const unsigned long TELEMETRY_INTERVAL_MS = 50;
unsigned long lastTelemetryMs = 0;

uint8_t pumpDir = 0;
uint8_t lastPumpSpeed = 0;

String inputLine;

void stopPump() {
  analogWrite(PUMP_PIN_A, 0);
  analogWrite(PUMP_PIN_B, 0);
}

void setPump(uint8_t speed, uint8_t dir) {
  if (speed == 0) {
    stopPump();
    return;
  }
  if (dir == 0) {
    analogWrite(PUMP_PIN_A, speed);
    analogWrite(PUMP_PIN_B, 0);
  } else {
    analogWrite(PUMP_PIN_A, 0);
    analogWrite(PUMP_PIN_B, speed);
  }
}

void updatePumpFromPot() {
  int potRaw = analogRead(POT_PIN);
  uint8_t speed = (potRaw <= POT_DEADBAND) ? 0 : (uint8_t)map(potRaw, 0, 1023, 0, 255);
  setPump(speed, pumpDir);
  lastPumpSpeed = speed;
}

void setValve(uint8_t idx, uint8_t state) {
  if (idx >= NUM_SECTORS) return;
  // state가 1(열림)이면 LOW를 쏴서 릴레이를 켜고, 0이면 HIGH를 쏴서 끕니다.
  digitalWrite(VALVE_PINS[idx], state ? LOW : HIGH);
}

void updateBuzzer(uint16_t sensorRaw[]) {
  bool shouldBeep = false;
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    if (sensorRaw[i] >= alarmRawThreshold[i]) {
      shouldBeep = true;
      break;
    }
  }
  if (shouldBeep && !buzzerActive) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    buzzerActive = true;
  } else if (!shouldBeep && buzzerActive) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
}

void sendReady() {
  Serial.print(F("READY,"));
  Serial.println(NUM_SECTORS);
}

void handleCommand(const String &line) {
  if (line.length() == 0) return;

  int firstComma = line.indexOf(',');
  String cmd = firstComma == -1 ? line : line.substring(0, firstComma);

  if (cmd == "VALVE") {
    int secondComma = line.indexOf(',', firstComma + 1);
    if (secondComma == -1) return;
    uint8_t idx = (uint8_t)line.substring(firstComma + 1, secondComma).toInt();
    uint8_t state = (uint8_t)line.substring(secondComma + 1).toInt();
    setValve(idx, state);
  } else if (cmd == "PUMP") {
    if (firstComma == -1) return;
    pumpDir = (uint8_t)line.substring(firstComma + 1).toInt();
  } else if (cmd == "ALARM") {
    int secondComma = line.indexOf(',', firstComma + 1);
    if (secondComma == -1) return;
    uint8_t idx = (uint8_t)line.substring(firstComma + 1, secondComma).toInt();
    if (idx >= NUM_SECTORS) return;
    alarmRawThreshold[idx] = (uint16_t)line.substring(secondComma + 1).toInt();
  } else if (cmd == "STATUS") {
    sendReady();
  }
}

void sendTelemetry(uint16_t sensorRaw[]) {
  Serial.print('T');
  Serial.print(',');
  Serial.print(millis());
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    Serial.print(',');
    Serial.print(sensorRaw[i]);
  }
  Serial.println();
}

void sendMotorStatus() {
  Serial.print('M');
  Serial.print(',');
  Serial.print(lastPumpSpeed);
  Serial.print(',');
  Serial.println(pumpDir);
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    pinMode(VALVE_PINS[i], OUTPUT);
    digitalWrite(VALVE_PINS[i], HIGH);  // 부팅 기본값 = 열림(배기, 잔압 방지)
  }

  pinMode(PUMP_PIN_A, OUTPUT);
  pinMode(PUMP_PIN_B, OUTPUT);
  stopPump();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  inputLine.reserve(64);
  sendReady();
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

  updatePumpFromPot();

  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;

    uint16_t sensorRaw[NUM_SECTORS];
    for (uint8_t i = 0; i < NUM_SECTORS; i++) {
      sensorRaw[i] = analogRead(SENSOR_PINS[i]);
    }

    sendTelemetry(sensorRaw);
    sendMotorStatus();
    updateBuzzer(sensorRaw);
  }
}
