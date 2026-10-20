#pragma once

#include "pixhawk_driver.h"
#include <stdint.h>

bool initcom ();
void send2drone (uint8_t id, float d );
void send2base (float d,Attitude a,GPS gps);
void send2all(float d );
