#pragma once
#include <stdint.h>

constexpr uint8_t DRONE_ID   = 2;  
const uint32_t PIXHAWK_BAUDRATE = 57600;
constexpr float max_speed = 6; //(m/s)

// Gestion des PINS :
constexpr uint8_t UWB_PIN_SCK  = 18;
constexpr uint8_t UWB_PIN_MISO = 19;
constexpr uint8_t UWB_PIN_MOSI = 23;
constexpr uint8_t UWB_PIN_SS   = 4;
constexpr uint8_t UWB_PIN_RST  = 27;
constexpr uint8_t UWB_PIN_IRQ  = 34;
constexpr int8_t  PIXHAWK_RX_PIN   = 21;
constexpr int8_t  PIXHAWK_TX_PIN   = 22; 
