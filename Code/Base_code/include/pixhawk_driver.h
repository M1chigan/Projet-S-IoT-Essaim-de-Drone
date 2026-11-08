#pragma once

#include <stdint.h>

struct __attribute__((packed)) Attitude {
  float roll, pitch, yaw;
};

struct __attribute__((packed)) GPS {
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  uint8_t fix_type;
  uint8_t satellites;
};

struct __attribute__((packed)) Altitude {
  float amsl_m;
};


bool initMAVLINK();
void pixhawk_update();
bool pixhawk_connected();
bool pixhawk_get_attitude (Attitude* a);
bool pixhawk_get_gps(GPS* gps);
bool pixhawk_get_altitude(Altitude* alt);




