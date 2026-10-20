#include "esp_now_driver.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"
#include "pixhawk_driver.h"
#include <stddef.h>

// Raccourci : Drone courant = drone ou on upload le code 
/*
Drone 1 (UWB_1) => ID = 1 & MAC = 0xCC, 0xDB, 0xA7, 0x06, 0x8E, 0x08
Drone 2 (UWB_2) => ID = 2 & MAC = 0xCC, 0xDB, 0xA7, 0x07, 0xB9, 0x78 
Drone 3 (UWB_3) => ID = 3 & MAC = 0x0C, 0xB8, 0x15, 0x6D, 0x5C, 0x30
Base (UWB_5) => ID = 5 & MAC = 0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50
*/

// ---------- Réglages ----------
const uint8_t  CHANNEL     = 1;
//-------------------------------

static uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t BASE_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50}; 
// Commenté la ligne correspond au drone courant
static uint8_t D1_MAC[6] = {0xCC, 0xDB, 0xA7, 0x06, 0x8E, 0x08}; 
static uint8_t D2_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50};
static uint8_t D3_MAC[6] = {0x0C, 0xB8, 0x15, 0x6D, 0x5C}; 

//Structure de la trame (attriibut packed pour éviter de modifier les arrangement d'octect)
typedef struct __attribute__((packed)) {
  uint8_t  id;
  uint32_t t_ms;
  float d;
  GPS gps; 
  Attitude a;
} Pkt;

static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {

  if (len == sizeof(Pkt)) {
    Pkt p;
    memcpy(&p, data, sizeof(p));
    Serial.printf("id=%u t=%lu ms d=%.3f\n", p.id, (unsigned long)p.t_ms, p.d);
  }

}

bool initcom() {

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  //On vérifie que c'est bien lancé 
  if (esp_now_init() != ESP_OK) {
    return false; 
  }

  esp_now_peer_info_t peer = {};
  peer.channel = CHANNEL;
  peer.encrypt = false;

  memcpy(peer.peer_addr, BCAST, 6); // Copier dans peer.peer_addr un tableau 
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, BASE_MAC, 6);
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, D1_MAC, 6); 
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, D2_MAC, 6);
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, D3_MAC, 6);
  esp_now_add_peer(&peer);

  // On appelle OnRecv quand message reçu
  esp_now_register_recv_cb(onRecv);
  return true; 
}

void send2drone(uint8_t id, float d) {
Pkt p;
p.id = DRONE_ID;
p.t_ms = millis();
p.d = d;

switch (id) {
    case 1:
      esp_now_send(D1_MAC, (uint8_t*)&p, sizeof(p));
      break;
    case 2:
      esp_now_send(D2_MAC, (uint8_t*)&p, sizeof(p));
      break;
    case 3:
      esp_now_send(D3_MAC, (uint8_t*)&p, sizeof(p));
      break;
}
}


void send2base(float d, Attitude a, GPS gps) {
    Pkt p;
    p.id = DRONE_ID;
    p.t_ms = millis();
    p.d = d;
    p.gps = gps;
    p.a = a;
    esp_now_send(BASE_MAC, (uint8_t*)&p, sizeof(p));
}

void send2all(float d ) {
  Pkt p;
  p.id = DRONE_ID;
  p.t_ms = millis();
  p.d = d;

  esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));

}



