// 모든 하드웨어 즉시 정지 (모터 정지 + 밸브 전부 닫힘) 후 대기
#include <Wire.h>

const uint8_t ADDR_VALVE = 0x20;

const uint8_t MOTOR2_IN1_PIN = 5;
const uint8_t MOTOR2_IN2_PIN = 6;
const uint8_t MOTOR1_IN1_PIN = 9;
const uint8_t MOTOR1_IN2_PIN = 10;

void pcfWrite(uint8_t addr, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);

  analogWrite(MOTOR1_IN1_PIN, 0);
  analogWrite(MOTOR1_IN2_PIN, 0);
  analogWrite(MOTOR2_IN1_PIN, 0);
  analogWrite(MOTOR2_IN2_PIN, 0);
  pcfWrite(ADDR_VALVE, 0x00); // VALVE_ACTIVE_HIGH=true 기준 전부 비에너자이즈(닫힘)

  Serial.println(F("전체 정지 완료 - 모터 OFF, 밸브 전부 닫힘"));
}

void loop() {
  // 아무 것도 안 함, 정지 상태 유지
}
