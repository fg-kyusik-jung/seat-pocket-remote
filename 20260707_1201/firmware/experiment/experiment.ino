// 좌판 경도 조절 시스템 - 실험용 펌웨어 (최소 구성: 1펌프 공용 + 섹터별 밸브/센서)
// 시리얼 프로토콜
//   Arduino -> PC : "T,<millis>,<sector0_raw>,<sector1_raw>,...\n"   (주기 전송, raw = analogRead 0~1023)
//                    "READY,<NUM_SECTORS>\n"                          (부팅 시 / STATUS 요청 시)
//   PC -> Arduino : "VALVE,<sector_idx>,<0|1>\n"   0=닫힘(대기 배기측 NO 개방 유지 아님, 밸브 OFF 상태) 1=열림
//                    "PUMP,<speed 0-255>,<dir 0|1>\n"
//                    "STATUS\n"
//
// 섹터 확장 시 NUM_SECTORS와 VALVE_PINS/SENSOR_PINS 배열만 늘리면 됨.

const uint8_t NUM_SECTORS = 1;

const uint8_t VALVE_PINS[NUM_SECTORS] = {2};
const uint8_t SENSOR_PINS[NUM_SECTORS] = {A0};

const uint8_t PUMP_PIN_A = 9;
const uint8_t PUMP_PIN_B = 10;

const unsigned long TELEMETRY_INTERVAL_MS = 50;
unsigned long lastTelemetryMs = 0;

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

void setValve(uint8_t idx, uint8_t state) {
  if (idx >= NUM_SECTORS) return;
  digitalWrite(VALVE_PINS[idx], state ? HIGH : LOW);
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
    int secondComma = line.indexOf(',', firstComma + 1);
    if (secondComma == -1) return;
    uint8_t speed = (uint8_t)constrain(line.substring(firstComma + 1, secondComma).toInt(), 0, 255);
    uint8_t dir = (uint8_t)line.substring(secondComma + 1).toInt();
    setPump(speed, dir);
  } else if (cmd == "STATUS") {
    sendReady();
  }
}

void sendTelemetry() {
  Serial.print('T');
  Serial.print(',');
  Serial.print(millis());
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    Serial.print(',');
    Serial.print(analogRead(SENSOR_PINS[i]));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    pinMode(VALVE_PINS[i], OUTPUT);
    digitalWrite(VALVE_PINS[i], LOW);
  }

  pinMode(PUMP_PIN_A, OUTPUT);
  pinMode(PUMP_PIN_B, OUTPUT);
  stopPump();

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

  unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
