#pragma once

#include "pixhawk_driver.h"
#include <stdint.h>

bool initcom ();
void send2drone (uint8_t id, uint8_t commande,float arg,float arg2 = 0);
void send2all(float d );
