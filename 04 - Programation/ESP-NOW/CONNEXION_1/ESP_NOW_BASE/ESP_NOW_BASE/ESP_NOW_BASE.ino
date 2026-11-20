#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t CHANNEL = 1;
uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Core Arduino-ESP32 3.x. Pour la 2.x : (const uint8_t* mac, const uint8_t* data, int len)
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  uint8_t hdr[3] = {0xAA, 0x55, (uint8_t)len};
  uint8_t cs = 0;
  for (int i = 0; i < len; i++) cs ^= data[i];
  Serial.write(hdr, 3);
  Serial.write(data, len);
  Serial.write(cs);
}

void setup() {
  Serial.begin(921600);       
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) while (true) delay(1000);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, BCAST, 6);
  p.channel = CHANNEL;
  p.encrypt = false;
  esp_now_add_peer(&p);

  esp_now_register_recv_cb(onRecv);
}

// Lecture des commandes envoyées par MATLAB (même format de trame)
void loop() {
  static uint8_t st = 0, n = 0, i = 0, buf[250], cs = 0;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (st) {
      case 0: st = (b == 0xAA) ? 1 : 0; break;
      case 1: st = (b == 0x55) ? 2 : 0; break;
      case 2: n = b; i = 0; cs = 0; st = (n > 0) ? 3 : 0; break;
      case 3: buf[i++] = b; cs ^= b; if (i == n) st = 4; break;
      case 4: if (b == cs) esp_now_send(BCAST, buf, n); st = 0; break;
    }
  }
}