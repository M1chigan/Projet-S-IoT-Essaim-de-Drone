// ESP32 -> simulateur ArduPilot, version ultra simple.
// Chaque appui sur le bouton BOOT envoie la commande suivante :
//   1) mode GUIDED   2) armer   3) décollage 10 m   4) atterrir   5) désarmer (puis on recommence)
//
// LED intégrée (GPIO2) :
//   allumée      = le drone est armé
//   éteinte      = le drone est désarmé
//   4 clignotements rapides = une commande a été REFUSÉE
//
// Le port USB sert à MAVLink (via bridge.py) : pas de moniteur série.

#include <common/mavlink.h>

#define LED_PIN 2
#define BTN_PIN 0

const uint8_t MY_SYSID = 1, MY_COMPID = MAV_COMP_ID_ONBOARD_COMPUTER;
const uint8_t TARGET_SYSID = 1, TARGET_COMPID = MAV_COMP_ID_AUTOPILOT1;

int nextStep = 0;
unsigned long lastHeartbeat = 0;
bool armedState = false;

void send(mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  Serial.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
}

void command(uint16_t cmd, float p1 = 0, float p2 = 0, float p7 = 0) {
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(MY_SYSID, MY_COMPID, &msg, TARGET_SYSID, TARGET_COMPID,
                                cmd, 0, p1, p2, 0, NAN, 0, 0, p7);
  send(msg);
}

void sendNextCommand() {
  switch (nextStep) {
    case 0: command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4); break; // GUIDED
    case 1: command(MAV_CMD_COMPONENT_ARM_DISARM, 1); break;                           // armer
    case 2: command(MAV_CMD_NAV_TAKEOFF, 0, 0, 10); break;                             // décollage 10 m
    case 3: command(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9); break; // LAND
    case 4: command(MAV_CMD_COMPONENT_ARM_DISARM, 0); break;                           // désarmer
  }
  nextStep = (nextStep + 1) % 5;
}

void blinkError() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, !armedState); delay(80);
    digitalWrite(LED_PIN, armedState);  delay(80);
  }
}

void readMavlink() {
  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial.available()) {
    if (!mavlink_parse_char(MAVLINK_COMM_0, Serial.read(), &msg, &status)) continue;

    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.sysid == TARGET_SYSID && msg.compid == TARGET_COMPID) {
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&msg, &hb);
      armedState = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
      digitalWrite(LED_PIN, armedState);
    }
    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      mavlink_command_ack_t ack;
      mavlink_msg_command_ack_decode(&msg, &ack);
      if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) blinkError();
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  // Heartbeat chaque seconde
  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MY_SYSID, MY_COMPID, &msg, MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    send(msg);
  }

  readMavlink();

  // Un appui sur BOOT = commande suivante
  if (digitalRead(BTN_PIN) == LOW) {
    sendNextCommand();
    while (digitalRead(BTN_PIN) == LOW) { readMavlink(); }   // attendre le relâchement
    delay(100);
  }
}
