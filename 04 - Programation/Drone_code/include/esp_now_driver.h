#pragma once
#include <Arduino.h>
#include "pixhawk_driver.h"
#include <stdint.h>

//----------------------------------------------
// --------------- Variables -------------------
//----------------------------------------------
enum : uint8_t { PKT_TELEM = 1, PKT_CONSOLE = 2 , PKT_ORDER = 3 };


//----------------------------------------------
// ----------- Fonction publiques --------------
//----------------------------------------------
bool initcom ();
void send2drone (uint8_t id, float d );
void send2base(float d, float z, const Attitude& a, const GPS& gps);
void send_msg2base(const char* msg);
void send2all(float d );
