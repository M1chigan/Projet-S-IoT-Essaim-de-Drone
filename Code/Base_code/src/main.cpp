#include "WString.h"
#include <Arduino.h>
#include <esp_now_driver.h>

String buffer = "";
int escState = 0; 
int DRONE_ID = 1;
float ALT_MAX = 70;

#define CMD_STATUS  1
#define CMD_MODE    2
#define CMD_TAKEOFF 3
#define CMD_ARM     4
#define CMD_GO      5
#define CMD_PARAM   6

struct ModeEntry { const char* name; uint8_t num; };
const ModeEntry MODES[] = {
  {"guided",    1},
  {"land",      2},
  {"rtl",       3},
  {"stabilize", 4},
  {"loiter",    5},
  {"auto",      6},
};

struct GOEntry { const char* name; uint8_t num; int8_t sign; };

const GOEntry GO[] = {
  {"gps",    1,  1},
  {"devant", 2,  1},
  {"nord",   3,  1},
  {"sud",    3, -1},
  {"est",    4,  1},
  {"ouest",  4, -1},  
  {"z",      5,  1},
  {"stop",   6,  1},
  {"tourne", 7,  1},
};

static int splitArgs(String s, String* out, int maxn) {
  int n = 0;
  s.trim();
  while (s.length() > 0 && n < maxn) {
    int sp = s.indexOf(' ');
    if (sp < 0) { out[n++] = s; break; }
    out[n++] = s.substring(0, sp);
    s = s.substring(sp + 1);
    s.trim();                      
  }
  return n;
}        

void executeCommand(String cmd) {
  String t[5];
  int n = splitArgs(cmd, t, 5);
  if (n == 0) return;

  String verbe = t[0];
  String arg   = t[1];
  String arg2  = t[2];
  verbe.toLowerCase();
  arg.toLowerCase();
  arg2.toLowerCase();


  // [1] status
  if (verbe == "status") {
    send2drone(DRONE_ID, CMD_STATUS, 0);
    Serial.println("Status demande");
    return;
  }

  // [2] mode <nom>
  if (verbe == "mode") {
    for (const auto& m : MODES) {
      if (arg == m.name) {
        send2drone(DRONE_ID, CMD_MODE, (float)m.num);
        Serial.printf("Mode %s envoye\n", m.name);
        return;
      }
    }
    Serial.println("Usage: mode <guided|land|rtl|stabilize|loiter|auto>");
    return;
  }

  // [3] takeoff <altitude>
  if (verbe == "takeoff") {
    float alt = arg.toFloat();
    if (alt > 0 && alt <= ALT_MAX) {
      send2drone(DRONE_ID, CMD_TAKEOFF, alt);
      Serial.printf("Decollage %.1f m envoye\n", alt);
    } else {
      Serial.printf("Usage: takeoff <altitude 0-%.0f>\n", ALT_MAX);
    }
    return;
  }

  if (verbe == "arm") {
    arg.toUpperCase();                        // accepte on, On, ON
    if (arg == "ON" || arg == "OFF") {
      bool on = (arg == "ON");
      send2drone(DRONE_ID, CMD_ARM, on ? 1 : 0);
      Serial.println(on ? "Armement envoye" : "Desarmement envoye");
    } else {
      Serial.println("Usage: arm <ON|OFF>");
    }
    return;
  }

  // [5] GO (à définir)
  if (verbe == "go") {
  

    for (const auto& m : GO) {
      if (arg == m.name) {
        send2drone(DRONE_ID, CMD_GO, (float)m.num, m.sign * arg2.toFloat());
        Serial.printf("GO %s %.1f m envoye\n", m.name, arg2.toFloat()); 
        return;
      }
    }
    Serial.println("GO: direction inconnue");
    return;
  }

  // help
  if (verbe == "help") {
    Serial.println("Commandes :\n"
                   " status\n"
                   " mode <guided|land|rtl|stabilize|loiter|auto>\n"
                   " takeoff <altitude>\n"
                   " arm <ON|OFF>\n"
                   " GO  <gps [GPS] |devant [distance]|nord [distance]|sud [distance]|est [distance]|ouest [distance]|z [hauteur]|STOP|Tourne [angle]>\n"
                   " help");
    return;
  }

  Serial.println("Commande inconnue (tape help)");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Console ESP32 prete. Tape 'help'.");
  initcom();
}

void loop() {
   while (Serial.available()) {
    char c = Serial.read();

    if (escState == 0 && c == 27) { escState = 1; continue; }
    if (escState == 1) { escState = (c == '[') ? 2 : 0; continue; }
    if (escState == 2) {
      if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '~') escState = 0;
      continue;
    }

    if (c == '\r' || c == '\n') {
      Serial.println();
      executeCommand(buffer);
      buffer = "";
      Serial.print("> ");
    } else if (c == 8 || c == 127) {          
      if (buffer.length() > 0) {
        buffer.remove(buffer.length() - 1);
        Serial.print("\b \b");
      }
    } else if (c >= 32) {          
      buffer += c;
      Serial.print(c);
    }
  }
}

