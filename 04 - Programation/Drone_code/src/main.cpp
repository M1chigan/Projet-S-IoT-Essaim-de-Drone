// Fichiers locaux
#include "config.h"
#include "filtre.h"
#include "pixhawk_driver.h"
#include "uwb_driver.h"
#include "esp_now_driver.h"
#include "console_driver.h"

// Librairies externes
#include <Arduino.h>
#include <HardwareSerial.h>

// ------------------------------------------------------------
//  Logs : en simulation l'USB transporte du MAVLink binaire,
//  donc AUCUN texte ne doit y passer. Utilise LOG/LOGF partout
//  (dans ce fichier ET dans uwb_driver, esp_now_driver, filtre...).
// ------------------------------------------------------------

#if SIM_MODE
#define LOG(x)
#define LOGF(...)
#else
  #define LOG(x)    Serial.println(x)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#endif

static float    last_distances[UWB_MAX_DRONES + 1] = {0.0f};
static uint32_t last_time_ms = 0;

static Attitude s_att;
static GPS      s_gps;
static uint32_t s_last_send2base = 0;

void setup() {
#if !SIM_MODE
    Serial.begin(115200);          // logs USB (en simulation, c'est initMAVLINK qui ouvre Serial)
#endif
    if (!initMAVLINK()) { LOG("MAVLINK Down"); } else { LOG("MAVLINK UP"); }

    if (!initUWB()) {
        LOG("UWB KO (ID invalide)");
#if !SIM_MODE
        ESP.restart();
#endif
    }

    if (!initcom()) { LOG("ESP-NOW Down"); } else { LOG("ESP-NOW Up"); }

    LOG("Pret");
    say("Drone en ligne et pret !");
    last_time_ms = millis();
}

void loop() {
    pixhawk_update();
    console_update();  
}
