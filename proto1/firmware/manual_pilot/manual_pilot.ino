// 좌판 경도 조절 시스템 - 매뉴얼 모드 하드웨어 검증용 파일럿 펌웨어
// (파일럿1 experiment.ino는 모터/센서 실험값 측정용, 이 파일은 엔코더+포켓 선택 UX 검증용)
//
// 하드웨어: 펌프모터 1(공용) + 엔코더 1(EC11, CLK/DT/SW) +
//           LED 2 + 솔레노이드밸브 2 + 압력센서 2 + 부저 1
//
// 동작:
//   1. 엔코더 스위치 롱프레스(>=LONG_PRESS_MS) -> 매뉴얼 모드 진입 (0번 포켓 선택, LED0 점등)
//   2. CW 회전 중(HOLD_TIMEOUT_MS 이내 연속 회전) -> 선택된 포켓 밸브 닫힘(펌프측 연결) + 펌프 최대속도 작동
//      최대압력 도달 시 펌프 정지 + 비프
//   3. CCW 회전 중 -> 선택된 포켓 밸브 열림(배기), 회전 멈추면 즉시 밸브 잠김
//      최저압력 도달 시 밸브 강제 잠금 + 비프
//   4. 스위치 숏프레스 -> 다음 포켓으로 전환 (LED 전환, 이전 포켓 출력 안전 정지)
//
// 시리얼 명령 (튜닝용, 실측 전 임시값):
//   "MAXTH,<idx>,<raw>\n"  포켓별 최대압력 raw 임계값 조정
//   "MINTH,<idx>,<raw>\n"  포켓별 최저압력 raw 임계값 조정
//
// 2026-07-14 셀렉터밸브(3way, D12) 추가했다가 원인불명 오작동으로 체크포인트 시점 기준 되돌림.
// 배선 처음부터 재구성 예정 (2way 밸브로 포켓 격리하는 방향으로 재검토 중).

const uint8_t NUM_SECTORS = 2;

const uint8_t VALVE_PINS[NUM_SECTORS] = {6, 7};
const uint8_t SENSOR_PINS[NUM_SECTORS] = {A0, A1};
const uint8_t LED_PINS[NUM_SECTORS] = {8, 11};

const uint8_t ENC_CLK_PIN = 2;  // 인터럽트 핀
const uint8_t ENC_DT_PIN = 3;
const uint8_t ENC_SW_PIN = 5;

const uint8_t BUZZER_PIN = 4;
const unsigned int BUZZER_FREQ_HZ = 440;

const uint8_t PUMP_PIN_A = 9;
const uint8_t PUMP_PIN_B = 10;
const uint8_t PUMP_FULL_SPEED = 255;  // 포텐셔미터 없이 항상 최대속도로 작동

// 펌프는 매뉴얼 모드에서 항상 "부풀리기" 한 방향으로만 작동 (배기는 밸브가 담당).
// 방향 0(정방향)이 맞음 - 파일럿1 실측으로 확정됨(2026-07-13).
const uint8_t PUMP_FIXED_DIR = 0;

// 파일럿1 실측 기준 확정값: 목표압력 상한 raw≈500 (40kPa까지 안 씀), 노이즈 허용폭 ±5
// (실측 후 MAXTH/MINTH 명령으로 언제든 재조정 가능)
uint16_t alarmMaxRawThreshold[NUM_SECTORS] = {500, 500};
uint16_t alarmMinRawThreshold[NUM_SECTORS] = {0, 0};

const unsigned long HOLD_TIMEOUT_MS = 200;   // 회전 신호가 이 시간 이상 없으면 "회전 멈춤"으로 간주
const unsigned long LONG_PRESS_MS = 1000;    // 이 이상 누르고 있으면 롱프레스
const unsigned long SW_DEBOUNCE_MS = 20;
const unsigned long ENCODER_DEBOUNCE_US = 5000;  // 펌프 PWM 노이즈로 인한 오인식 방지 위해 상향(2026-07-14)
const unsigned long DEBUG_PRINT_INTERVAL_MS = 200;  // 센서 raw 값 확인용 디버그 출력 주기
unsigned long lastDebugPrintMs = 0;

volatile int8_t encoderStepAccum = 0;
volatile unsigned long lastEncoderInterruptMicros = 0;

bool systemOn = false;
uint8_t selectedPocket = 0;

unsigned long lastCwStepMs = 0 - HOLD_TIMEOUT_MS - 1;
unsigned long lastCcwStepMs = 0 - HOLD_TIMEOUT_MS - 1;

// 회전 시작 시 첫 펄스는 노이즈로 반대 방향이 찍히는 경우가 있어, 같은 방향 두 번째 펄스가
// 와야 실제 회전으로 확정함 (첫 펄스는 "대기" 상태로만 기록).
int8_t pendingStepDir = 0;  // 0=없음, 1=CW 대기중, -1=CCW 대기중
unsigned long pendingStepMs = 0;

bool swPrevState = HIGH;
unsigned long swLastChangeMs = 0;
unsigned long swPressStartMs = 0;

bool buzzerActive = false;

String inputLine;

void encoderISR() {
  unsigned long now = micros();
  if (now - lastEncoderInterruptMicros < ENCODER_DEBOUNCE_US) return;
  lastEncoderInterruptMicros = now;

  // 실측 결과 물리적 CW가 반대로 인식되어 부호 반전 (2026-07-14)
  if (digitalRead(ENC_DT_PIN)) {
    encoderStepAccum--;
  } else {
    encoderStepAccum++;
  }
}

void stopPump() {
  analogWrite(PUMP_PIN_A, 0);
  analogWrite(PUMP_PIN_B, 0);
}

void setPump(uint8_t speed) {
  if (speed == 0) {
    stopPump();
    return;
  }
  if (PUMP_FIXED_DIR == 0) {
    analogWrite(PUMP_PIN_A, speed);
    analogWrite(PUMP_PIN_B, 0);
  } else {
    analogWrite(PUMP_PIN_A, 0);
    analogWrite(PUMP_PIN_B, speed);
  }
}

void setValve(uint8_t idx, uint8_t state) {
  // state가 1(열림)이면 LOW를 쏴서 릴레이를 켜고, 0이면 HIGH를 쏴서 끕니다.
  digitalWrite(VALVE_PINS[idx], state ? LOW : HIGH);
}

void setAllValvesClosed() {
  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    setValve(i, 0);
  }
}

void selectPocket(uint8_t idx) {
  stopPump();
  setAllValvesClosed();
  digitalWrite(LED_PINS[selectedPocket], LOW);
  selectedPocket = idx;
  digitalWrite(LED_PINS[selectedPocket], HIGH);
  lastCwStepMs = millis() - HOLD_TIMEOUT_MS - 1;
  lastCcwStepMs = millis() - HOLD_TIMEOUT_MS - 1;
  pendingStepDir = 0;
}

void powerOn() {
  systemOn = true;
  digitalWrite(LED_PINS[selectedPocket], HIGH);
}

void handleSwitch() {
  bool swNow = digitalRead(ENC_SW_PIN);
  unsigned long now = millis();

  // 버튼을 누르고 있는 도중 롱프레스 임계값을 넘는 순간 즉시 반응 (떼는 걸 기다리지 않음)
  if (!systemOn && swNow == LOW && swPrevState == LOW && (now - swPressStartMs) >= LONG_PRESS_MS) {
    powerOn();
  }

  if (now - swLastChangeMs < SW_DEBOUNCE_MS) return;

  if (swNow == LOW && swPrevState == HIGH) {
    swPressStartMs = now;
    swLastChangeMs = now;
  } else if (swNow == HIGH && swPrevState == LOW) {
    unsigned long heldMs = now - swPressStartMs;
    swLastChangeMs = now;

    if (!systemOn) {
      if (heldMs >= LONG_PRESS_MS) {
        powerOn();
      }
    } else {
      if (heldMs < LONG_PRESS_MS) {
        selectPocket((selectedPocket + 1) % NUM_SECTORS);
      }
    }
  }
  swPrevState = swNow;
}

void updateBuzzer(bool shouldBeep) {
  if (shouldBeep && !buzzerActive) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    buzzerActive = true;
  } else if (!shouldBeep && buzzerActive) {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
}

void handleCommand(const String &line) {
  if (line.length() == 0) return;

  int firstComma = line.indexOf(',');
  String cmd = firstComma == -1 ? line : line.substring(0, firstComma);

  if (cmd == "MAXTH" || cmd == "MINTH") {
    int secondComma = line.indexOf(',', firstComma + 1);
    if (secondComma == -1) return;
    uint8_t idx = (uint8_t)line.substring(firstComma + 1, secondComma).toInt();
    if (idx >= NUM_SECTORS) return;
    uint16_t raw = (uint16_t)line.substring(secondComma + 1).toInt();
    if (cmd == "MAXTH") {
      alarmMaxRawThreshold[idx] = raw;
    } else {
      alarmMinRawThreshold[idx] = raw;
    }
  }
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_SECTORS; i++) {
    pinMode(VALVE_PINS[i], OUTPUT);
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(VALVE_PINS[i], LOW);  // 비활성(안전 기본값) = 밸브 닫힘
    digitalWrite(LED_PINS[i], LOW);
  }

  pinMode(PUMP_PIN_A, OUTPUT);
  pinMode(PUMP_PIN_B, OUTPUT);
  stopPump();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), encoderISR, FALLING);

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

  handleSwitch();

  if (!systemOn) {
    return;
  }

  noInterrupts();
  int8_t delta = encoderStepAccum;
  encoderStepAccum = 0;
  interrupts();

  unsigned long now = millis();
  bool wasHoldingCw = (now - lastCwStepMs) < HOLD_TIMEOUT_MS;
  bool wasHoldingCcw = (now - lastCcwStepMs) < HOLD_TIMEOUT_MS;

  if (delta > 0) {
    if (wasHoldingCw) {
      lastCwStepMs = now;
    } else if (pendingStepDir == 1 && (now - pendingStepMs) < HOLD_TIMEOUT_MS) {
      lastCwStepMs = now;
      pendingStepDir = 0;
    } else {
      pendingStepDir = 1;
      pendingStepMs = now;
    }
  } else if (delta < 0) {
    if (wasHoldingCcw) {
      lastCcwStepMs = now;
    } else if (pendingStepDir == -1 && (now - pendingStepMs) < HOLD_TIMEOUT_MS) {
      lastCcwStepMs = now;
      pendingStepDir = 0;
    } else {
      pendingStepDir = -1;
      pendingStepMs = now;
    }
  }

  bool holdingCw = (now - lastCwStepMs) < HOLD_TIMEOUT_MS;
  bool holdingCcw = (now - lastCcwStepMs) < HOLD_TIMEOUT_MS;

  uint16_t sensorRaw = analogRead(SENSOR_PINS[selectedPocket]);
  bool atMax = sensorRaw >= alarmMaxRawThreshold[selectedPocket];
  bool atMin = sensorRaw <= alarmMinRawThreshold[selectedPocket];

  bool shouldInflate = holdingCw && !atMax;
  bool shouldDeflate = holdingCcw && !atMin;

  setValve(selectedPocket, shouldDeflate ? 1 : 0);

  if (shouldInflate) {
    setPump(PUMP_FULL_SPEED);
  } else {
    stopPump();
  }

  updateBuzzer(atMax || atMin);

  if (now - lastDebugPrintMs >= DEBUG_PRINT_INTERVAL_MS) {
    lastDebugPrintMs = now;
    Serial.print(F("POCKET,"));
    Serial.print(selectedPocket);
    Serial.print(F(",RAW,"));
    Serial.print(sensorRaw);
    Serial.print(F(",MAXTH,"));
    Serial.print(alarmMaxRawThreshold[selectedPocket]);
    Serial.print(F(",ATMAX,"));
    Serial.print(atMax);
    Serial.print(F(",ATMIN,"));
    Serial.print(atMin);
    Serial.print(F(",S1,"));
    Serial.print(digitalRead(ENC_CLK_PIN));
    Serial.print(F(",S2,"));
    Serial.print(digitalRead(ENC_DT_PIN));
    Serial.print(F(",STEPACCUM,"));
    Serial.println(delta);
  }
}
