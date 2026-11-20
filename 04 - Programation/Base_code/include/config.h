#pragma once
#include <stdint.h>
 
constexpr float max_speed = 6; // Utiliser dans le Innovation Gate Filter(m/s)

//----------------------------------------------
// ------------ Gestion des PIN ----------------
//----------------------------------------------
constexpr int8_t  PIXHAWK_RX_PIN   = 21; 
constexpr int8_t  PIXHAWK_TX_PIN   = 22; 

//---------------------------------------------
// ------------ Communications ----------------
//---------------------------------------------
const uint8_t  CHANNEL     = 1; //Channel Wifi pour ESP-NOW

static uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; 
static uint8_t BASE_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50}; 
static uint8_t D1_MAC[6] = {0xCC, 0xDB, 0xA7, 0x06, 0x8E, 0x08}; 
static uint8_t D2_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0xB9, 0x78};
static uint8_t D3_MAC[6] = {0x0C, 0xB8, 0x15, 0x6D, 0x5C, 0x30 };


