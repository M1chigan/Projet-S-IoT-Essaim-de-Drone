#pragma once

#include <stdint.h>

struct Attitude {
    float roll;
    float pitch;
    float yaw;
};

struct GPS {
    int32_t lat_e7;      // latitude  en degrés x 10^7  (ex : 48,5839° -> 485839000)
    int32_t lon_e7;      // longitude en degrés x 10^7
    int32_t alt_mm;      // altitude en mm (au-dessus du niveau de la mer)
    uint8_t fix_type;    // 0-1 = pas de fix, 2 = 2D, 3 = 3D (ou mieux)
    uint8_t satellites;  // nombre de satellites visibles
};

bool initMAVLINK();
void pixhawk_update();
bool pixhawk_connected();
bool pixhawk_get_attitude (Attitude* a);
bool pixhawk_get_gps(GPS* gps);



