#pragma once
#include <stdint.h>

//----------------------------------------------
// --------------- Variables -------------------
//----------------------------------------------
struct Attitude { float roll = 0, pitch = 0, yaw = 0; };           // radians
struct GPS      { int32_t lat_e7 = 0, lon_e7 = 0, alt_mm = 0;
                  uint8_t fix_type = 0, satellites = 0; };
struct Altitude { float amsl_m = 0; };                              // metres


// ---------------------------------------------
// ----------- Fonction publiques --------------
//----------------------------------------------
bool initMAVLINK();
void pixhawk_update();  

bool pixhawk_connected();
bool pixhawk_armed();
uint32_t pixhawk_mode();   
bool pixhawk_get_attitude(Attitude* a);   // true si nouvelle donnee
bool pixhawk_get_gps(GPS* gps);
bool pixhawk_get_altitude(Altitude* alt);
bool pixhawk_get_ack(uint16_t* command, uint8_t* result);  // result 0 = accepte

void process_order(uint8_t commande, float arg,float arg2=0);

void move_body(float avant = 0, float droite = 0, float bas = 0);
void move_ned(float nord = 0, float est = 0, float bas = 0);
void set_velocity_body(float vx = 0, float vy = 0, float vz = 0);
void set_yaw(float deg, bool relatif = true, bool horaire = true);

void sim_report(const char* name, float value);

void pixhawk_arm(bool on);
void pixhawk_set_mode(uint8_t custom_mode);
void pixhawk_takeoff(float alt_m);