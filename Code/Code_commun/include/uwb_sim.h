#pragma once
#include <stdint.h>
 
// Appelee par pixhawk_update() quand une distance UWB simulee arrive
// (message NAMED_VALUE_FLOAT "d<id>" envoye par bridge_uwb.py).
void uwb_sim_push(uint8_t from_id, float dist_m);