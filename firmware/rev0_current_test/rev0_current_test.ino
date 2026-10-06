// rev0 최악부하 전류 실측용 스트레스 테스트 (릴레이+PCF8574, 8비트, 0x20)
// 모터 2개 전속 연속구동 + 밸브 6개(PCF8574 0x20, P0~P5) 2초 간격 전부 열기/닫기 반복
// 이 상태에서 SMPS(+) 라인에 멀티미터(전류모드) 직렬로 물려서 최대전류 측정

#include <Wire.h>

const uint8_t ADDR_VALVE = 0x20;

const uint8_t MOTOR2_IN1_PIN = 5;
const uint8_t MOTOR2_IN2_PIN = 6;
const uint8_t MOTOR1_IN1_PIN = 9;
const uint8_t MOTOR1_IN2_PIN = 10;
const uint8_t PUMP_FIXED_DIR = 0; // 파일럿1 실측치 가정

const bool VALVE_ACTIVE_HIGH = true; // rev0 확정값
const uint8_t VALVE_BITS_MASK = 0b00111111; // P0~P5 6채널

const uint16_t INTERVAL_MS = 2000;

void pcfWrite(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

void runPump(uint8_t pinA, uint8_t pinB, uint8_t speed) {
  if (PUMP_FIXED_DIR == 0) {
    analogWrite(pinA, speed);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, speed);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);

  uint8_t closedVal = VALVE_ACTIVE_HIGH ? 0x00 : 0xFF;
  pcfWrite(ADDR_VALVE, closedVal); // 밸브 전부 닫힘(비에너자이즈)으로 시작

  runPump(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 255);
  runPump(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, 255);
  Serial.println(F("모터 2개 전속 연속구동 시작 (rev0 릴레이 기준)"));
}

void loop() {
  uint8_t openVal = VALVE_ACTIVE_HIGH ? VALVE_BITS_MASK : (uint8_t)~VALVE_BITS_MASK;
  uint8_t closedVal = VALVE_ACTIVE_HIGH ? 0x00 : 0xFF;

  pcfWrite(ADDR_VALVE, openVal);
  Serial.println(F("밸브 6개 전부 OPEN (모터는 계속 구동중)"));
  delay(INTERVAL_MS);

  pcfWrite(ADDR_VALVE, closedVal);
  Serial.println(F("밸브 6개 전부 CLOSE (모터는 계속 구동중)"));
  delay(INTERVAL_MS);
}
