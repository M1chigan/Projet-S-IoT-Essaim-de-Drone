#pragma once

#include "pixhawk_driver.h"
#include <stdint.h>

bool initcom ();
void send2drone (uint8_t id, uint8_t commande,float variable);
void send2all(float d );
