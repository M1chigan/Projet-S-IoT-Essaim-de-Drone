#include "pixhawk_driver.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <mavlink.h>
#include "config.h"

// ---------------------------------------------------------------
//  Port utilise pour MAVLink : un SEUL port, pour lecture ET ecriture
// ---------------------------------------------------------------
#if SIM_MODE
  static HardwareSerial& s_serial = Serial;    // USB -> bridge.py
#else
  static HardwareSerial& s_serial = Serial2;   // GPIO -> Pixhawk
#endif

// ---------------------------------------------------------------
//  Constantes
// ---------------------------------------------------------------
static constexpr size_t   RX_BUFFER_SIZE      = 1024;
static constexpr uint16_t MAX_BYTES_PER_UPDATE = 256;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;   // notre heartbeat
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 3000;  // perte du drone
static constexpr uint32_t REQUEST_PERIOD_MS   = 2000;   // re-demande si silence

static constexpr uint8_t  OUR_COMPONENT_ID = MAV_COMP_ID_ONBOARD_COMPUTER;

// Modes de la console -> custom_mode ArduCopter
// 1 guided, 2 land, 3 rtl, 4 stabilize, 5 loiter, 6 auto
static const uint8_t MODE_TO_ARDU[7] = { 0, 4, 9, 6, 0, 5, 3 };

// ---------------------------------------------------------------
//  Etat interne
// ---------------------------------------------------------------
static bool     s_connected   = false;
static uint8_t  s_target_comp = MAV_COMP_ID_AUTOPILOT1;
static uint32_t s_last_heartbeat_rx = 0;
static uint32_t s_last_heartbeat_tx = 0;
static bool     s_armed = false;
static uint32_t s_mode  = 0;

static Attitude s_att;  static bool s_att_new = false;
static GPS      s_gps;  static bool s_gps_new = false;
static Altitude s_alt;  static bool s_alt_new = false;

static uint16_t s_ack_cmd = 0;  static uint8_t s_ack_res = 0;
static bool     s_ack_new = false;

// Messages que l'on veut recevoir en continu
enum { S_ATT = 0, S_POS = 1, S_GPS = 2, S_COUNT = 3 };
struct StreamReq {
  uint16_t msgid;
  float    hz;
  uint32_t timeout_ms;   // si rien recu depuis ce delai -> on re-demande
  uint32_t last_rx;
  uint32_t last_req;
};
static StreamReq s_streams[S_COUNT] = {
  { MAVLINK_MSG_ID_ATTITUDE,            10.0f, 1000, 0, 0 },
  { MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  5.0f, 1000, 0, 0 },
  { MAVLINK_MSG_ID_GPS_RAW_INT,          2.0f, 2000, 0, 0 },
};

// ---------------------------------------------------------------
//  Envoi
// ---------------------------------------------------------------
static void sendMessage(mavlink_message_t& msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  s_serial.write(buf, len);
}

static void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(OUR_SYSID, OUR_COMPONENT_ID, &msg,
                             MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  sendMessage(msg);
}

static void sendCommand(uint16_t cmd, float p1 = 0, float p2 = 0, float p3 = 0,
                        float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(OUR_SYSID, OUR_COMPONENT_ID, &msg,
                                TARGET_SYSID, s_target_comp,
                                cmd, 0, p1, p2, p3, p4, p5, p6, p7);
  sendMessage(msg);
}

static void requestMessage(uint16_t message_id, float frequency_hz) {
  sendCommand(MAV_CMD_SET_MESSAGE_INTERVAL,
              (float)message_id, 1000000.0f / frequency_hz);
}

// ---------------------------------------------------------------
//  Initialisation
// ---------------------------------------------------------------
bool initMAVLINK() {
  s_serial.setRxBufferSize(RX_BUFFER_SIZE);   // AVANT begin()
#if SIM_MODE
  s_serial.begin(PIXHAWK_BAUDRATE);           // USB : pas de broches
#else
  s_serial.begin(PIXHAWK_BAUDRATE, SERIAL_8N1, PIXHAWK_RX_PIN, PIXHAWK_TX_PIN);
#endif
  return true;
}

// ---------------------------------------------------------------
//  Boucle de mise a jour
// ---------------------------------------------------------------
void pixhawk_update() {
  uint32_t now = millis();

  // 1) Notre heartbeat, TOUJOURS (meme avant d'avoir vu le drone)
  if (now - s_last_heartbeat_tx >= HEARTBEAT_PERIOD_MS) {
    s_last_heartbeat_tx = now;
    sendHeartbeat();
  }

  // 2) Lecture et decodage
  mavlink_message_t msg;
  mavlink_status_t  status;
  uint16_t budget = MAX_BYTES_PER_UPDATE;

  while (budget-- > 0 && s_serial.available()) {
    uint8_t c = s_serial.read();
    if (!mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) continue;

    // On ne garde que le drone pilote (ignore drones 2/3, MAVProxy, etc.)
    if (msg.sysid != TARGET_SYSID) continue;

    uint32_t t = millis();
    switch (msg.msgid) {

      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.autopilot == MAV_AUTOPILOT_INVALID) break;   // pas l'autopilote
        s_target_comp       = msg.compid;
        s_last_heartbeat_rx = t;
        s_connected         = true;
        s_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
        s_mode  = hb.custom_mode;
        break;
      }

      case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&msg, &a);
        s_att.roll = a.roll;  s_att.pitch = a.pitch;  s_att.yaw = a.yaw;
        s_att_new = true;
        s_streams[S_ATT].last_rx = t;
        break;
      }

      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g;
        mavlink_msg_gps_raw_int_decode(&msg, &g);
        s_gps.lat_e7 = g.lat;  s_gps.lon_e7 = g.lon;  s_gps.alt_mm = g.alt;
        s_gps.fix_type = g.fix_type;
        s_gps.satellites = g.satellites_visible;
        s_gps_new = true;
        s_streams[S_GPS].last_rx = t;
        break;
      }

      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&msg, &gp);
        s_alt.amsl_m = gp.alt / 1000.0f;            // mm -> m
        s_alt_new = true;
        s_streams[S_POS].last_rx = t;
        break;
      }

      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(&msg, &ack);
        s_ack_cmd = ack.command;
        s_ack_res = ack.result;
        s_ack_new = true;
        break;
      }

      default: break;
    }
  }

  // 3) Perte de connexion
  now = millis();
  if (s_connected && now - s_last_heartbeat_rx > HEARTBEAT_TIMEOUT_MS) {
    s_connected = false;
  }

  // 4) (Re)demande des flux de telemetrie tant qu'ils ne arrivent pas
  if (s_connected) {
    for (int i = 0; i < S_COUNT; i++) {
      StreamReq& s = s_streams[i];
      if (now - s.last_rx > s.timeout_ms && now - s.last_req >= REQUEST_PERIOD_MS) {
        s.last_req = now;
        requestMessage(s.msgid, s.hz);
      }
    }
  }
}

// ---------------------------------------------------------------
//  Acces aux donnees
// ---------------------------------------------------------------
bool     pixhawk_connected() { return s_connected; }
bool     pixhawk_armed()     { return s_armed; }
uint32_t pixhawk_mode()      { return s_mode; }

bool pixhawk_get_attitude(Attitude* a) {
  if (!s_att_new) return false;
  s_att_new = false;  *a = s_att;  return true;
}
bool pixhawk_get_gps(GPS* g) {
  if (!s_gps_new) return false;
  s_gps_new = false;  *g = s_gps;  return true;
}
bool pixhawk_get_altitude(Altitude* a) {
  if (!s_alt_new) return false;
  s_alt_new = false;  *a = s_alt;  return true;
}
bool pixhawk_get_ack(uint16_t* command, uint8_t* result) {
  if (!s_ack_new) return false;
  s_ack_new = false;  *command = s_ack_cmd;  *result = s_ack_res;  return true;
}

// ---------------------------------------------------------------
//  Ordres de la console
// ---------------------------------------------------------------
void process_order(uint8_t commande, float variable) {
  switch (commande) {

    case CMD_ARM:
      sendCommand(MAV_CMD_COMPONENT_ARM_DISARM, 1);
      break;

    case CMD_MODE: {
      int m = (int)(variable + 0.5f);
      if (m >= 1 && m <= 6) {
        sendCommand(MAV_CMD_DO_SET_MODE,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODE_TO_ARDU[m]);
      }
      break;
    }

    case CMD_TAKEOFF:   // exige : mode GUIDED + drone arme
      if (variable > 0 && variable <= 30) {          // borne de securite
        sendCommand(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, NAN, 0, 0, variable);
      }
      break;

    default: break;
  }
}