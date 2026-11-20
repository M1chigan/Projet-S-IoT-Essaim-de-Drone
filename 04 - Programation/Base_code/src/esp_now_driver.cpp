#include "esp_now_driver.h"
#include "config.h"


#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"
#include "pixhawk_driver.h"
#include <stddef.h>


//------------------------------------------------
//               Structure Trame
//------------------------------------------------

struct __attribute__((packed)) Pkt {
  uint8_t  type;    // PKT_TELEM
  uint8_t  id;
  uint32_t t_ms;
  float    d, z;
  Attitude a;
  GPS      gps;
};

struct __attribute__((packed)) Pkt_base {
  uint8_t  type;       // PKT_ORDER
  uint8_t  id;        
  uint32_t t_ms;
  uint8_t  commande;
  float    arg, arg2;
};

struct __attribute__((packed)) Pkt_console {
  uint8_t  type;       // PKT_CONSOLE
  uint8_t  id;
  uint32_t t_ms;
  char     msg[32];   
};         



// ---------------------------------------------
// ---------------- Fonctions ------------------
//----------------------------------------------


static Pkt s_last[8];


//Code s'activant quand l'ESP-NOW reçoit un message. 
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1) return;

  switch (data[0]) {                       

    case PKT_TELEM: {
      if (len != sizeof(Pkt)) return;
      Pkt p;
      memcpy(&p, data, sizeof(p));
      if (p.id < 8) s_last[p.id] = p;     
      break;
    }

    case PKT_CONSOLE: {
      if (len != sizeof(Pkt_console)) return;
      Pkt_console p;
      memcpy(&p, data, sizeof(p));
      p.msg[sizeof(p.msg) - 1] = '\0';    
      Serial.printf("[drone %u] %s\n", p.id, p.msg);
      break;
    }

    default:
      break;                   
  }
}

//Initialation des communications
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

  // Appel OnRecv quand message reçu
  esp_now_register_recv_cb(onRecv);
  return true; 
}

// Fonction qui permet d'envoyer un drone précisément
bool send2drone(uint8_t id, uint8_t commande, float arg, float arg2) {
    static const uint8_t* const MACS[] = { nullptr, D1_MAC, D2_MAC, D3_MAC };
    if (id < 1 || id > 3) return false;               // drone inconnu

    Pkt_base p = {};                                  // pas d'octets aléatoires
    p.type     = PKT_ORDER;
    p.id       = id;                                  // le drone visé, pas la base
    p.t_ms     = millis();
    p.commande = commande;
    p.arg      = arg;
    p.arg2     = arg2;

    return esp_now_send(MACS[id], (uint8_t*)&p, sizeof(p)) == ESP_OK;
}

// Fonction qui permet d'envoyer à tous les ESP
void send2all(uint8_t commande,float arg,float arg2) {
   Pkt_base p = {};                                  // pas d'octets aléatoires
    p.type     = PKT_ORDER;                              
    p.t_ms     = millis();
    p.commande = commande;
    p.arg      = arg;
    p.arg2     = arg2;

  esp_now_send(BCAST, (uint8_t*)&p, sizeof(p));
}



