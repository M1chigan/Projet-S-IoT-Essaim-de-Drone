#pragma once

#include "pixhawk_driver.h"
#include <stdint.h>



//----------------------------------------------
// --------------- Variables -------------------
//----------------------------------------------
enum : uint8_t { PKT_TELEM = 1, PKT_CONSOLE = 2 , PKT_ORDER =3 };


// ---------------------------------------------
// ----------- Fonction publiques --------------
//----------------------------------------------
bool initcom ();
bool send2drone(uint8_t id, uint8_t commande, float arg, float arg2=0);
void send2all(float d );
