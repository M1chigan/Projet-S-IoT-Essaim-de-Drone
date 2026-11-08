// Fichiers locaux
#include "config.h"
#include "filtre.h"
#include "pixhawk_driver.h"
#include "uwb_driver.h"
#include "esp_now_driver.h"

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

    // MAVLink en PREMIER : le heartbeat doit partir quoi qu'il arrive
    if (!initMAVLINK()) { LOG("MAVLINK Down"); } else { LOG("MAVLINK UP"); }

    if (!initUWB()) {
        LOG("UWB KO (ID invalide)");
#if !SIM_MODE
        ESP.restart();
#endif
    }

    if (!initcom()) { LOG("ESP-NOW Down"); } else { LOG("ESP-NOW Up"); }

    LOG("Pret");
    last_time_ms = millis();
}

void loop() {
    // 1) Pixhawk : heartbeat + lecture. A appeler A CHAQUE TOUR, sans delay().
    pixhawk_update();

    if (pixhawk_connected()) {
        pixhawk_get_attitude(&s_att);   // met a jour s_att si nouvelle donnee
        pixhawk_get_gps(&s_gps);

        // Envoi a la base a 5 Hz max (au lieu d'un delay(50) bloquant)
        if (millis() - s_last_send2base >= 200) {
            s_last_send2base = millis();
        }
    }

    // 2) Communication + UWB
    scheduler_update();
    uwb_update();

    // 3) Traitement des distances UWB
    uint8_t from_id = 0;
    float raw_dist = 0.0f;

    if (uwb_get_distance(&from_id, &raw_dist)) {
        uint32_t current_time_ms = millis();
        float dt = (current_time_ms - last_time_ms) / 1000.0f;
        last_time_ms = current_time_ms;
        if (dt <= 0.0f) dt = 0.02f;

        float processed_dist = raw_dist;
        polynomial_filter(&processed_dist);

        if (from_id >= 1 && from_id <= UWB_MAX_DRONES) {
            float prev_d = last_distances[from_id];
            if (prev_d == 0.0f) prev_d = processed_dist;

            float max_relative_speed = 3.0f;

            if (innovationGate_filter(processed_dist, prev_d, max_relative_speed, dt)) {
                last_distances[from_id] = processed_dist;
                LOGF("From ID: %d | Raw: %.3f m -> Filtered: %.3f m\n",
                     from_id, raw_dist, processed_dist);
            } else {
                LOGF("REJECTED by Gate: From ID %d -> %.3f m (prev: %.3f m)\n",
                     from_id, processed_dist, prev_d);
            }
        }
    }
}
