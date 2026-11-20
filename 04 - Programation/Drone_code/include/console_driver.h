#pragma once
#include <Arduino.h>

// Codes de commande (console -> ESP32)
#define CMD_STATUS  1
#define CMD_MODE    2
#define CMD_TAKEOFF 3
#define CMD_ARM     4
#define CMD_GO      5
#define CMD_PARAM   6



//----------------------------------------------
// ------------ Gestion des PIN ----------------
//----------------------------------------------

struct __attribute__((packed)) Pkt_console {
  uint8_t  type;       // PKT_CONSOLE
  uint8_t  id;         // Identifiant
  uint32_t t_ms;       // Timestamp
  char     msg[32];    // texte terminé par '\0'
};         

//----------------------------------------------
// ----------- Fonction publiques --------------
//----------------------------------------------
void process_order(uint8_t commande, float arg,float arg2);
void console_update();
void say(const char* fmt, ...);