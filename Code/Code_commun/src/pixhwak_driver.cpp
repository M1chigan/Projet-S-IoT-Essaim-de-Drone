#include "esp32-hal.h"
#include "pixhawk_driver.h"

#include <Arduino.h>
#include <mavlink.h>
#include <HardwareSerial.h>
#include "config.h"


static HardwareSerial& s_serial = Serial2;
static constexpr size_t   RX_BUFFER_SIZE   = 1024;

static constexpr uint8_t OUR_SYSTEM_ID    = 1;
static constexpr uint8_t OUR_COMPONENT_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

static constexpr float    ATTITUDE_RATE_HZ      = 10.0f;
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS  = 3000; 
static constexpr uint32_t ATTITUDE_TIMEOUT_MS   = 1000;  
static constexpr uint32_t REQUEST_PERIOD_MS     = 2000; 
static constexpr uint16_t MAX_BYTES_PER_UPDATE  = 256;
static constexpr float    ALTITUDE_RATE_HZ    = 5.0f;
static constexpr uint32_t ALTITUDE_TIMEOUT_MS = 1000;

static bool     s_connected = false;
static uint8_t  s_target_sys  = 1;
static uint8_t  s_target_comp = MAV_COMP_ID_AUTOPILOT1;
 
static uint32_t s_last_heartbeat_ms = 0;
static uint32_t s_last_attitude_ms  = 0;
static uint32_t s_last_gps_ms  = 0;
static uint32_t s_last_request_ms   = 0;
static uint32_t s_last_alt_ms         = 0;
static uint32_t s_last_request_alt_ms = 0;


 
static Attitude s_att = {0, 0, 0};
static bool     s_att_new = false;

static GPS s_gps = {0, 0, 0};
static bool s_gps_new = false;

static Altitude s_alt = {0};
static bool     s_alt_new = false;

static void requestMessage(uint16_t message_id, float frequency_hz) {
    float interval_us = 1000000.0f / frequency_hz;
 
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
    mavlink_msg_command_long_pack(
        OUR_SYSTEM_ID, OUR_COMPONENT_ID, &msg,
        s_target_sys, s_target_comp,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,                      // confirmation
        (float)message_id,      // param 1 : identifiant du message
        interval_us,            // param 2 : intervalle en microsecondes
        0, 0, 0, 0, 0           // params 3 à 7 : inutilisés
    );
 
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    s_serial.write(buf, len);
}


bool initMAVLINK() {
    s_serial.setRxBufferSize(RX_BUFFER_SIZE);   // doit être appelé AVANT begin()
    s_serial.begin(PIXHAWK_BAUDRATE, SERIAL_8N1, PIXHAWK_RX_PIN, PIXHAWK_TX_PIN);
    return true;
}


void pixhawk_update() {
    mavlink_message_t msg;
    mavlink_status_t  status;
 
    uint16_t budget = MAX_BYTES_PER_UPDATE;
    while (budget-- > 0 && s_serial.available()) {
        uint8_t c = s_serial.read();
 
        if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) continue;
 
        switch (msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                if (hb.autopilot == MAV_AUTOPILOT_INVALID) break;  
 
                s_target_sys        = msg.sysid;
                s_target_comp       = msg.compid;
                s_last_heartbeat_ms = millis();
                s_connected         = true;
                break;
            }
            case MAVLINK_MSG_ID_ATTITUDE: {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                s_att.roll  = att.roll;
                s_att.pitch = att.pitch;
                s_att.yaw   = att.yaw;
                s_att_new   = true;
                s_last_attitude_ms = millis();
                break;
            }
            case MAVLINK_MSG_ID_GPS_RAW_INT: {
                mavlink_gps_raw_int_t gps;
                mavlink_msg_gps_raw_int_decode(&msg, &gps);
                
                s_gps.lat_e7     = gps.lat;
                s_gps.lon_e7     = gps.lon;
                s_gps.alt_mm     = gps.alt;
                s_gps.fix_type   = gps.fix_type;
                s_gps.satellites = gps.satellites_visible;
                s_gps_new        = true;
                s_last_gps_ms    = millis();

                break;
            }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                mavlink_global_position_int_t gp;
                mavlink_msg_global_position_int_decode(&msg, &gp);

                s_alt.amsl_m  = gp.alt / 1000.0f;   // mm -> m
                s_alt_new     = true;
                s_last_alt_ms = millis();
                break;
}
            default:
                break;
        }
    }
 
    uint32_t now = millis();
    if (s_connected && now - s_last_heartbeat_ms > HEARTBEAT_TIMEOUT_MS) {
        s_connected = false;
    }
 
    if (s_connected &&
        now - s_last_attitude_ms > ATTITUDE_TIMEOUT_MS &&
        now - s_last_request_ms  >= REQUEST_PERIOD_MS) {
        requestMessage(MAVLINK_MSG_ID_ATTITUDE, ATTITUDE_RATE_HZ);
        s_last_request_ms = now;
    }
    if (s_connected &&
        now - s_last_alt_ms         > ALTITUDE_TIMEOUT_MS &&
        now - s_last_request_alt_ms >= REQUEST_PERIOD_MS) {
        requestMessage(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, ALTITUDE_RATE_HZ);
        s_last_request_alt_ms = now;
}
}
 
bool pixhawk_connected() {
    return s_connected;
}
 
bool pixhawk_get_attitude(Attitude* a) {
    if (!s_att_new) return false;
    s_att_new = false;
    *a = s_att;
    return true;
}

bool pixhawk_get_gps (GPS* gps) {
    if (!s_gps_new) return false;
    s_gps_new = false;
    *gps = s_gps;
    return true;
}

bool pixhawk_get_altitude(Altitude* alt) {
    if (!s_alt_new) return false;
    s_alt_new = false;
    *alt = s_alt;
    return true;
}