// ULN2803 -> LED 3ch 순차 점멸 테스트
// D2/D3/D4 -> ULN2803 IN1/IN2/IN3 -> OUT1/OUT2/OUT3 -> LED1/LED2/LED3
// 시리얼(115200)로 HIGH/LOW 전환을 실시간 출력

const uint8_t PINS[3] = {2, 3, 4};
const uint16_t INTERVAL_MS = 1000;

void printState(uint8_t ledIndex, uint8_t state) {
  Serial.print("LED");
  Serial.print(ledIndex + 1);
  Serial.print(" (D");
  Serial.print(PINS[ledIndex]);
  Serial.print(") -> ");
  Serial.println(state == HIGH ? "HIGH" : "LOW");
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], LOW);
  }
}

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PINS[i], HIGH);
    printState(i, HIGH);
    delay(INTERVAL_MS);
    digitalWrite(PINS[i], LOW);
    printState(i, LOW);
    delay(INTERVAL_MS);
  }
}
