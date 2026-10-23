//Fichier locaux
#include "config.h"
#include "filtre.h"
#include "pixhawk_driver.h"
#include "uwb_driver.h"
#include "esp_now_driver.h"

//Librairie externe
#include <Arduino.h>
#include <HardwareSerial.h>

static float last_distances[UWB_MAX_DRONES + 1] = {0.0f};
static uint32_t last_time_ms = 0;

void setup (){
    Serial.begin(115200);

    if (!initUWB()) {
        Serial.println("UWB KO (ID invalide)");
        ESP.restart();
    }

    if (!initcom()) {
        Serial.println("ESP-NOW Down");
    } else Serial.println("ESP-NOW Up");

    if (!initMAVLINK()){
        Serial.println("MAVLINK Down");
    } else {
        Serial.println("ESP-MAVLINK UP");
    }
    Serial.println("Pret");
    last_time_ms = millis();
}

void loop() {
    /*
    pixhawk_update();

   if (pixhawk_connected()) {
        
        Attitude att;
        if (pixhawk_get_attitude(&att)) {
            Serial.print("C'est parti ! ");
        }

        GPS gpsData;
        if (pixhawk_get_gps(&gpsData)) {
            Serial.print("[GPS] Lat: ");
            Serial.print(gpsData.lat_e7 / 1e7, 7);
            Serial.print(" | Lon: ");
            Serial.print(gpsData.lon_e7 / 1e7, 7);
            Serial.print(" | Alt: ");
            Serial.print(gpsData.alt_mm / 1000.0);
            Serial.print("m | Sats: ");
            Serial.println(gpsData.satellites);
        }
        send2base(2.5,att,gpsData);

    } else {
        Serial.println("Recherche de la Pixhawk (pas de Heartbeat)...");
        delay(1000);
    }
    
    delay(50);
    */

    scheduler_update();
    uwb_update();

    uint8_t from_id = 0;
    float raw_dist = 0.0f;

    if (uwb_get_distance(&from_id, &raw_dist)) {
        uint32_t current_time_ms = millis();
        float dt = (current_time_ms - last_time_ms) / 1000.0f;
        last_time_ms = current_time_ms;

        // Prevent dt from being zero or negative on fast loops
        if (dt <= 0.0f) dt = 0.02f; 

        // --- FILTER CHAIN ---

        float processed_dist = raw_dist;
        polynomial_filter(&processed_dist);

        if (from_id >= 1 && from_id <= UWB_MAX_DRONES) {
            float prev_d = last_distances[from_id];

            if (prev_d == 0.0f) {
                prev_d = processed_dist;
            }

            float max_relative_speed = 3.0f; 

            if (innovationGate_filter(processed_dist, prev_d, max_relative_speed, dt)) {
                last_distances[from_id] = processed_dist;

                Serial.printf("From ID: %d | Raw: %.3f m -> Filtered: %.3f m\n", 
                              from_id, raw_dist, processed_dist);
                
            } else {
                // Measurement rejected by the gate (sudden unrealistic jump)
                Serial.printf("REJECTED by Gate: From ID %d tried to jump to %.3f m (prev: %.3f m)\n", 
                              from_id, processed_dist, prev_d);
            }
        }
    }
    

}

