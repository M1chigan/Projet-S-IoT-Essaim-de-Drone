#include "WString.h"
#include <Arduino.h>
#include <esp_now_driver.h>

// put function declarations here:
String buffer = "";
int escState = 0; 
int DRONE_ID = 1;
float ALT_MAX = 70;

#define CMD_STATUS  1
#define CMD_MODE    2
#define CMD_TAKEOFF 3
#define CMD_ARM     4
#define CMD_PARAM   5

struct ModeEntry { const char* name; uint8_t num; };
const ModeEntry MODES[] = {
  {"guided",    1},
  {"land",      2},
  {"rtl",       3},
  {"stabilize", 4},
  {"loiter",    5},
  {"auto",      6},
};

void executeCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;

  // Découpe : premier mot = verbe, reste = argument
  int sp = cmd.indexOf(' ');
  String verbe = (sp < 0) ? cmd : cmd.substring(0, sp);
  String arg   = (sp < 0) ? ""  : cmd.substring(sp + 1);
  arg.trim();

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

  // [4] arm <on|off>
  if (verbe == "arm") {
    if (arg == "on" || arg == "off") {
      send2drone(DRONE_ID, CMD_ARM, arg == "on" ? 1 : 0);
      Serial.println(arg == "on" ? "Armement envoye" : "Desarmement envoye");
    } else {
      Serial.println("Usage: arm <on|off>");
    }
    return;
  }

  // [5] param (à définir)
  if (verbe == "param") {
    Serial.println("Param : pas encore defini");
    return;
  }

  // help
  if (verbe == "help") {
    Serial.println("Commandes :\n"
                   " status\n"
                   " mode <guided|land|rtl|stabilize|loiter|auto>\n"
                   " takeoff <altitude>\n"
                   " arm <on|off>\n"
                   " param (a definir)\n"
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
    } else if (c == 8 || c == 127) {          // retour arrière
      if (buffer.length() > 0) {
        buffer.remove(buffer.length() - 1);
        Serial.print("\b \b");
      }
    } else if (c >= 32) {                      // caractères imprimables uniquement
      buffer += c;
      Serial.print(c);
    }
  }
}

