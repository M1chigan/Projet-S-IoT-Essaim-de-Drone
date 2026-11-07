#include <MAVLink.h>

#define MAV_SERIAL Serial          // test via USB + Bridge.py
// #define MAV_SERIAL Serial2      // vrai drone : GPIO16/17 vers le Pixhawk
#define MAV_BAUD 115200

#define SYSID 200
#define COMPID 191
#define TARGET_SYS 1
#define TARGET_COMP 1

void sendMsg(mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAV_SERIAL.write(buf, len);
}

void logMsg(const char *fmt, ...) {
  char text[50];
  va_list ap; va_start(ap, fmt);
  vsnprintf(text, sizeof(text), fmt, ap);
  va_end(ap);
  mavlink_message_t msg;
  mavlink_msg_statustext_pack(SYSID, COMPID, &msg, MAV_SEVERITY_INFO, text, 0, 0);
  sendMsg(msg);
}

void sendHeartbeat() {
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(SYSID, COMPID, &msg, MAV_TYPE_ONBOARD_CONTROLLER,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
  sendMsg(msg);
}

void cmd(uint16_t id, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
         float p5 = 0, float p6 = 0, float p7 = 0) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(SYSID, COMPID, &msg, TARGET_SYS, TARGET_COMP,
                                id, 0, p1, p2, p3, p4, p5, p6, p7);
  sendMsg(msg);
}

// --- État du drone, mis à jour par les messages reçus ---
bool droneSeen = false, armed = false, gpsOk = false;
uint32_t droneMode = 0;
float relAlt = 0;                  // altitude relative en m

// --- Machine à états ---
enum State { WAIT_DRONE, WAIT_GPS, SET_GUIDED, ARMING, TAKING_OFF, HOVER, RTL_SENT, DONE };
int state = WAIT_DRONE;
uint32_t tState = 0, tRetry = 0;

void goTo(int s) { state = s; tState = millis(); tRetry = 0; }

void runState() {
  uint32_t now = millis();
  switch (state) {
    case WAIT_DRONE:
      if (droneSeen) { logMsg("ESP32: drone detecte"); goTo(WAIT_GPS); }
      break;
    case WAIT_GPS:
      if (gpsOk && now - tState > 5000) { logMsg("ESP32: GPS OK"); goTo(SET_GUIDED); }
      break;
    case SET_GUIDED:
      if (droneMode == 4) { logMsg("ESP32: GUIDED OK"); goTo(ARMING); break; }
      if (now - tRetry > 2000) { tRetry = now;
        cmd(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4); }
      break;
    case ARMING:
      if (armed) { logMsg("ESP32: ARMED"); goTo(TAKING_OFF); break; }
      if (now - tRetry > 2000) { tRetry = now; cmd(MAV_CMD_COMPONENT_ARM_DISARM, 1); }
      break;
    case TAKING_OFF:
      if (relAlt > 9.0) { logMsg("ESP32: alt atteinte"); goTo(HOVER); break; }
      if (now - tRetry > 3000) { tRetry = now;
        cmd(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 10); }
      break;
    case HOVER:
      if (now - tState > 10000) {      // vol stationnaire 10 s
        logMsg("ESP32: RTL");
        cmd(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6);
        goTo(RTL_SENT);
      }
      break;
    case RTL_SENT:
    case DONE:
      break;
  }
}

void readMavlink() {
  mavlink_message_t msg; mavlink_status_t st;
  while (MAV_SERIAL.available()) {
    if (!mavlink_parse_char(MAVLINK_COMM_0, MAV_SERIAL.read(), &msg, &st)) continue;
    if (msg.sysid != TARGET_SYS) continue;       // on ignore les autres émetteurs

    switch (msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb; mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.type == MAV_TYPE_GCS) break;
        droneSeen = true;
        droneMode = hb.custom_mode;
        armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        break; }
      case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g; mavlink_msg_gps_raw_int_decode(&msg, &g);
        gpsOk = g.fix_type >= 3;
        break; }
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t p; mavlink_msg_global_position_int_decode(&msg, &p);
        relAlt = p.relative_alt / 1000.0f;
        break; }
      case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack; mavlink_msg_command_ack_decode(&msg, &ack);
        logMsg("ACK cmd=%u res=%u", ack.command, ack.result);
        break; }
    }
  }
}

void setup() {
  MAV_SERIAL.begin(MAV_BAUD);
  // Serial2 : MAV_SERIAL.begin(MAV_BAUD, SERIAL_8N1, 16, 17);
}

void loop() {
  static uint32_t tHb = 0;
  if (millis() - tHb >= 1000) { tHb = millis(); sendHeartbeat(); }
  readMavlink();
  runState();
}