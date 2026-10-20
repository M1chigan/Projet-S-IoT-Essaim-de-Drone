

void uwb_update();
bool uwb_get_distance ();

#pragma once

#include <stdint.h>

constexpr uint8_t UWB_MAX_DRONES = 3;

bool initUWB();
void uwb_update();
bool uwb_start_ranging();
bool uwb_is_idle();
uint8_t uwb_last_initiator();
uint32_t uwb_ms_since_activity();
bool uwb_get_distance(uint8_t* from_id, float* d);
void scheduler_update();
