// 모터는 정지 상태로 두고 밸브 6개(PCF8574 0x20, P0~P5)만 2초 간격 전부 열기/닫기 반복
#include <Wire.h>

const uint8_t ADDR_VALVE = 0x20;
const uint8_t MOTOR2_IN1_PIN = 5;
const uint8_t MOTOR2_IN2_PIN = 6;
const uint8_t MOTOR1_IN1_PIN = 9;
const uint8_t MOTOR1_IN2_PIN = 10;

const bool VALVE_ACTIVE_HIGH = true;
const uint8_t VALVE_BITS_MASK = 0b00111111;
const uint16_t INTERVAL_MS = 50; // 10Hz (50ms on/off)

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

  uint8_t closedVal = VALVE_ACTIVE_HIGH ? 0x00 : 0xFF;
  pcfWrite(ADDR_VALVE, closedVal);
  Serial.println(F("모터 정지 상태 - 밸브만 테스트"));
}

void loop() {
  uint8_t openVal = VALVE_ACTIVE_HIGH ? VALVE_BITS_MASK : (uint8_t)~VALVE_BITS_MASK;
  uint8_t closedVal = VALVE_ACTIVE_HIGH ? 0x00 : 0xFF;

  pcfWrite(ADDR_VALVE, openVal);
  delay(INTERVAL_MS);

  pcfWrite(ADDR_VALVE, closedVal);
  delay(INTERVAL_MS);
}
