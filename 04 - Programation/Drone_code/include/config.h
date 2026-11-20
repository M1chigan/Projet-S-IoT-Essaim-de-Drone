#pragma once
#include <stdint.h>

constexpr uint8_t DRONE_ID   = 1;  


#define TARGET_SYSID  1
#define OUR_SYSID     201
#define SIM_MODE 1
#define SIM_SYSID 250   
#if SIM_MODE
const uint32_t PIXHAWK_BAUDRATE = 115200;
#else
const uint32_t PIXHAWK_BAUDRATE = 57600;
#endif

constexpr float max_speed = 6; 


//----------------------------------------------
// ------------ Gestion des PIN ----------------
//----------------------------------------------
constexpr uint8_t UWB_PIN_SCK  = 18;
constexpr uint8_t UWB_PIN_MISO = 19;
constexpr uint8_t UWB_PIN_MOSI = 23;
constexpr uint8_t UWB_PIN_SS   = 4;
constexpr uint8_t UWB_PIN_RST  = 27;
constexpr uint8_t UWB_PIN_IRQ  = 34;
constexpr int8_t  PIXHAWK_RX_PIN   = 21;
constexpr int8_t  PIXHAWK_TX_PIN   = 22; 

//---------------------------------------------
// ------------ Communications ----------------
//---------------------------------------------


