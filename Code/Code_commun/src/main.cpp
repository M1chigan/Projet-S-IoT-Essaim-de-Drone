//Fichier locaux
#include "config.h"
#include "pixhawk_driver.h"
#include "uwb_driver.h"
#include "esp_now_driver.h"

//Librairie externe
#include <Arduino.h>
#include <HardwareSerial.h>

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
    
}

void loop() {
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

}

