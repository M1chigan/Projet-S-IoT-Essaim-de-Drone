#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- Réglages ----------
const uint8_t  CHANNEL     = 1;      // identique sur tous les ESP32
const uint8_t  FIRST_ID    = 1;      // id du premier drone simulé
const uint8_t  NB_DRONES   = 3;      // mets 3, 5... pour simuler un essaim
const uint32_t PERIOD_MS   = 20;     // 50 Hz
// ------------------------------

uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t BASE_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50}; 

typedef struct __attribute__((packed)) {
  uint8_t  id;
  uint32_t t_ms;
  float ax, ay, az, gx, gy, gz;
} Pkt;                                // 29 octets

// Bruit gaussien approximatif (somme de 4 tirages uniformes)
float noise(float amp) {
  float s = 0;
  for (int i = 0; i < 4; i++) s += random(-1000, 1001) / 1000.0f;
  return amp * s / 4.0f;
}

// Réception des commandes de la base
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  Serial.printf("Commande recue (%d octets) :", len);
  for (int i = 0; i < len; i++) Serial.printf(" %u", data[i]);
  Serial.println();
}

Pkt simulate(uint8_t id) {
  float t  = millis() / 1000.0f;
  float ph = id * 0.7f;               // déphasage entre drones

  Pkt p;
  p.id   = id;
  p.t_ms = millis();
  // Accéléromètre (m/s²) : gravité + oscillation lente + vibrations moteurs
  p.ax = 0.8f * sinf(0.5f * t + ph) + noise(0.15f);
  p.ay = 0.8f * cosf(0.5f * t + ph) + noise(0.15f);
  p.az = 9.81f + 0.3f * sinf(2.0f * t + ph) + noise(0.20f);
  // Gyroscope (rad/s) : petites rotations + bruit
  p.gx = 0.10f * sinf(0.8f * t + ph) + noise(0.01f);
  p.gy = 0.10f * cosf(0.8f * t + ph) + noise(0.01f);
  p.gz = 0.05f * sinf(0.3f * t + ph) + noise(0.01f);
  return p;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW KO");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BASE_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onRecv);

  Serial.printf("Simulateur pret : %d drone(s), ids %d a %d\n",
                NB_DRONES, FIRST_ID, FIRST_ID + NB_DRONES - 1);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= PERIOD_MS) {
    last = millis();
    for (uint8_t k = 0; k < NB_DRONES; k++) {
      Pkt p = simulate(FIRST_ID + k);
      esp_now_send(BASE_MAC, (uint8_t*)&p, sizeof(p));
      delay(2);                 
    }
  }
}