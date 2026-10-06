// rev1 최악부하 전류 실측용 스트레스 테스트
// 모터 2개 전속 연속구동 + 밸브 6개(PCF8575 0x20, P00~P05) 2초 간격 전부 열기/닫기 반복
// 이 상태에서 SMPS(+) 라인에 멀티미터(전류모드) 직렬로 물려서 최대전류 측정

#include <Wire.h>

const uint8_t ADDR_IOVALVE = 0x20;

const uint8_t MOTOR2_IN1_PIN = 5;
const uint8_t MOTOR2_IN2_PIN = 6;
const uint8_t MOTOR1_IN1_PIN = 9;
const uint8_t MOTOR1_IN2_PIN = 10;
const uint8_t PUMP_FIXED_DIR = 0; // 파일럿1 실측치 가정

const uint16_t VALVE_BITS_MASK = 0b0000000000111111; // P00~P05 6채널
const uint16_t INTERVAL_MS = 2000;

void pcfWrite16(uint8_t addr, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(lowByte(value));
  Wire.write(highByte(value));
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
  pcfWrite16(ADDR_IOVALVE, 0x0000); // 밸브 전부 닫힘으로 시작

  runPump(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 255);
  runPump(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, 255);
  Serial.println(F("모터 2개 전속 연속구동 시작"));
}

void loop() {
  pcfWrite16(ADDR_IOVALVE, VALVE_BITS_MASK);
  Serial.println(F("밸브 6개 전부 OPEN (모터는 계속 구동중)"));
  delay(INTERVAL_MS);

  pcfWrite16(ADDR_IOVALVE, 0x0000);
  Serial.println(F("밸브 6개 전부 CLOSE (모터는 계속 구동중)"));
  delay(INTERVAL_MS);
}
