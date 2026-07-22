// I2C 주소 스캐너 — LCD(0x27 예상), PCF8574 확장모듈 2개(0x20~0x26 중 점퍼로 설정) 확인용.
// 배선 후 이 스케치를 올리고 시리얼 모니터(115200)를 열면 3초마다 재스캔.
// 장치를 하나씩만 연결해서 주소를 확인하거나, 전부 연결한 채로 몇 개가 잡히는지 확인해도 됨.

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial) {}
  Serial.println(F("I2C 스캐너 시작"));
}

const char *hint(uint8_t addr) {
  if (addr == 0x27) return "LCD 예상 주소";
  if (addr == 0x20 || addr == 0x21) return "PCF8574 확장모듈 예상 주소";
  if (addr >= 0x20 && addr <= 0x26) return "PCF8574 계열 주소대역 (점퍼로 여기까지 조정 가능)";
  return "";
}

void loop() {
  Serial.println(F("---- 스캔 시작 ----"));
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found++;
      Serial.print(F("발견: 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      const char *h = hint(addr);
      if (h[0] != '\0') {
        Serial.print(F("  ("));
        Serial.print(h);
        Serial.print(F(")"));
      }
      Serial.println();
    }
  }

  if (found == 0) {
    Serial.println(F("장치 없음 - 배선(SDA/SCL/VCC/GND) 확인할 것"));
  } else {
    Serial.print(F("총 "));
    Serial.print(found);
    Serial.println(F("개 발견"));
  }

  Serial.println();
  delay(3000);
}
