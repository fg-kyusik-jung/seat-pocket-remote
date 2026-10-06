// 좌판측 브리지 (XIAO ESP32-C3): 리모컨-ESP32와 ESP-NOW로 통신, 우노와는 UART로 그냥 바이트 중계만 함
// (프로토콜 해석 없음 - dumb pipe). 우노 SoftwareSerial(D8=RX,D11=TX,9600bps)과 여기 D6/D7을 연결.
// 주의: 우노 TX(5V) -> 브리지 RX(3.3V) 방향은 저항분배기(예: 220Ω+440Ω)로 전압 낮춰서 연결할 것.
//
// ⚠ 리모컨-ESP32 쪽 remote_esp32.ino의 BRIDGE_MAC을 이 보드의 실제 MAC 주소로 채워넣어야 함
//   (아래 부팅 시 시리얼 출력되는 MAC 확인). 이 파일의 REMOTE_MAC도 마찬가지로 리모컨의 실제 MAC으로 채워야 함.
//
// (2026-10-06 수정) 원래는 리모컨이 먼저 ESP-NOW로 뭔가 보내야만 그 MAC을 자동 등록하는 구조였는데,
// 리모컨이 엔코더/스위치를 안 건드리면 아무것도 먼저 안 보내서 우노의 부팅시 메시지(BL,0 등)를
// 브리지가 받고도 보낼 곳이 없어 버려지는 버그가 있었음 - REMOTE_MAC을 고정으로 미리 등록해서 해결.

#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

HardwareSerial UnoLink(1); // UART1
const uint8_t UNO_TX_PIN = D6; // -> 우노 D8(RX)
const uint8_t UNO_RX_PIN = D7; // <- 우노 D11(TX), 저항분배기 경유

// ---------- 대상 리모컨 MAC (실측 후 교체) ----------
uint8_t REMOTE_MAC[6] = {0x44, 0xB1, 0x76, 0x5F, 0x34, 0xF4};

// PING 수신 시 플래그만 세워두고, 실제 PONG 전송은 메인 루프에서 처리한다.
// (2026-10-07 발견) 수신 콜백은 메인 루프와 다른 태스크에서 실행되는데, 그 안에서 바로
// esp_now_send()를 호출(PONG 응답)하니 매번 100% 전송 실패하는 현상이 실측으로 확인됨 -
// 우노→리모컨 중계(loop() 안에서 송신, 정상 동작)와 비교해서 찾아낸 원인. 리모컨측 LCD 깨짐 때와
// 같은 "콜백에서 무거운/민감한 작업 직접 하지 말 것" 원칙을 여기도 적용.
volatile bool pendingPong = false;

// 리모컨 -> (ESP-NOW 수신) -> 우노 (UART 그대로 전달)
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == 4 && memcmp(data, "PING", 4) == 0) {
    pendingPong = true;
    return;
  }
  Serial.print("[RX from remote] len=");
  Serial.print(len);
  Serial.print(" data=");
  for (int i = 0; i < len; i++) Serial.print((char)data[i]);
  Serial.println();
  UnoLink.write(data, len);
  UnoLink.write('\n');
}

// ESP-NOW는 전달을 보장하지 않아서(2026-10-06 실측으로 확인 - LCD/백라이트 명령이 간헐적으로
// 유실되어 리모컨 화면이 따로 노는 증상 발생), 전송 콜백으로 실제 도달 여부를 확인하고 실패 시 재전송한다.
volatile bool sendCbDone = false;
volatile bool sendCbSuccess = false;

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  sendCbSuccess = (status == ESP_NOW_SEND_SUCCESS);
  sendCbDone = true;
}

bool sendToRemote(const String &msg) {
  const uint8_t MAX_RETRY = 4;
  for (uint8_t attempt = 0; attempt < MAX_RETRY; attempt++) {
    sendCbDone = false;
    sendCbSuccess = false;
    esp_err_t r = esp_now_send(REMOTE_MAC, (const uint8_t *)msg.c_str(), msg.length());
    if (r != ESP_OK) continue;

    unsigned long start = millis();
    while (!sendCbDone && (millis() - start) < 20) { delay(1); }

    if (sendCbDone && sendCbSuccess) return true;
  }
  Serial.print("[TX to remote] ");
  Serial.print(msg);
  Serial.println(" (전송 최종 실패)");
  return false;
}

void setup() {
  Serial.begin(115200);
  UnoLink.begin(9600, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // WiFi 절전모드가 ESP-NOW 수신을 간헐적으로 지연/누락시키는 문제 방지
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, REMOTE_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_err_t peerResult = esp_now_add_peer(&peer);
  Serial.print("리모컨 peer 등록 결과: ");
  Serial.println(peerResult == ESP_OK ? "성공" : "실패");

  Serial.print("브리지 시작. 내 MAC(리모컨측 BRIDGE_MAC에 이 값을 넣을 것): ");
  Serial.println(WiFi.macAddress());
}

String lineBuf;

void loop() {
  // PING에 대한 PONG 응답은 여기(메인 루프)에서만 보낸다
  if (pendingPong) {
    pendingPong = false;
    bool ok = sendToRemote("PONG");
    Serial.println(ok ? "[PING->PONG] 응답 성공" : "[PING->PONG] 응답 최종 실패");
  }

  // 우노 -> (UART) -> 리모컨 (ESP-NOW 전송), 줄 단위로 모아서 전송
  while (UnoLink.available() > 0) {
    char c = (char)UnoLink.read();
    if (c == '\n') {
      if (lineBuf.length() > 0) {
        Serial.print("[UART from uno] ");
        Serial.println(lineBuf);
        sendToRemote(lineBuf);
      }
      lineBuf = "";
    } else if (c != '\r') {
      lineBuf += c;
    }
  }
}
