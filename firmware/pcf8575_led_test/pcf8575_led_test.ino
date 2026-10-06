// PCF8575(0x20) 출력으로 LED 3개 순차 점멸 - HIGH 출력(약한 풀업 소싱) 밝기 확인용
// P17/P16/P15 -> 저항 -> LED -> GND, HIGH=ON
// 시리얼(115200)로 상태 전환 실시간 출력

#include <Wire.h>

const uint8_t ADDR_PCF = 0x20;
const uint8_t BIT_LED[3] = {15, 14, 13}; // P17, P16, P15
const uint16_t INTERVAL_MS = 1000;

void pcfWrite16(uint16_t value) {
  Wire.beginTransmission(ADDR_PCF);
  Wire.write(lowByte(value));
  Wire.write(highByte(value));
  Wire.endTransmission();
}

void printState(uint8_t ledIndex, bool on) {
  Serial.print("LED");
  Serial.print(ledIndex + 1);
  Serial.print(" (P1");
  Serial.print(7 - ledIndex);
  Serial.print(") -> ");
  Serial.println(on ? "HIGH" : "LOW");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pcfWrite16(0x0000);
}

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    pcfWrite16((uint16_t)1 << BIT_LED[i]);
    printState(i, true);
    delay(INTERVAL_MS);
    pcfWrite16(0x0000);
    printState(i, false);
    delay(INTERVAL_MS);
  }
}
