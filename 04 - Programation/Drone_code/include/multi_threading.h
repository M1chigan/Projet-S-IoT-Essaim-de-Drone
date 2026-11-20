#pragma once
#include <stdint.h>
#include "pixhawk_driver.h" 


//----------------------------------------------
// ----------- Fonction publiques --------------
//----------------------------------------------
bool rtos_peek_attitude(Attitude* a);
bool rtos_peek_gps(GPS* g);

