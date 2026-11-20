// ---------------------------------------------
// ---------------- Desciption -----------------
//----------------------------------------------
// Code permettant de traiter les ordres envoyés 
// par la base. 
//----------------------------------------------


#include "console_driver.h"
#include "config.h"

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include "pixhawk_driver.h"      // pixhawk_arm, move_body, ...
#include "esp_now_driver.h"      // send_msg2base

// ---------------------------------------------------------------
//  Limites de securite et tables
// ---------------------------------------------------------------
static constexpr float MAX_TAKEOFF_M = 30.0f;
static constexpr float MAX_MOVE_M    = 50.0f;
static constexpr float MAX_Z_M       = 30.0f;
static constexpr float MAX_TURN_DEG  = 360.0f;

// Modes de la console -> custom_mode ArduCopter
static const uint8_t MODE_TO_ARDU[7]   = { 0, 4, 9, 6, 0, 5, 3 };
static const char*   MODE_NAME[7]      = { "?", "GUIDED", "LAND", "RTL",
                                           "STABILIZE", "LOITER", "AUTO" };
// Types de CMD_GO (colonne "num" de la table GO de la base)
static const char*   GO_NAME[8]        = { "?", "GPS", "devant", "nord",
                                           "est", "z", "STOP", "tourne" };

// ---------------------------------------------------------------
//  Retour vers la base (31 caracteres maximum)
// ---------------------------------------------------------------
void say(const char* fmt, ...) {
  char txt[32];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(txt, sizeof(txt), fmt, ap);
  va_end(ap);
  send_msg2base(txt);
}

// ---------------------------------------------------------------
//  CMD_GO : renvoie true si l'ordre a ete envoye, false s'il est hors limites
//  (aucun ACK MAVLink pour les deplacements)
// ---------------------------------------------------------------
static bool go_exec(int type, float v) {
  switch (type) {
    case 2:  if (fabsf(v) > MAX_MOVE_M)   return false; move_body(v, 0, 0);       return true;
    case 3:  if (fabsf(v) > MAX_MOVE_M)   return false; move_ned(v, 0, 0);        return true;
    case 4:  if (fabsf(v) > MAX_MOVE_M)   return false; move_ned(0, v, 0);        return true;
    case 5:  if (fabsf(v) > MAX_Z_M)      return false; move_body(0, 0, -v);      return true;  // NED : bas positif
    case 6:  set_velocity_body(0, 0, 0);                                          return true;
    case 7:  if (fabsf(v) > MAX_TURN_DEG) return false; set_yaw(fabsf(v), true, v >= 0); return true;
    default: return false;
  }
}

// ---------------------------------------------------------------
//  Execution d'un ordre
// ---------------------------------------------------------------
void process_order(uint8_t commande, float arg, float arg2) {
  // Pas de heartbeat du Pixhawk : on n'envoie rien et on le dit
  //if (!pixhawk_connected()) {
  //  say("Pixhawk non connecte");
  //  return;
  //}

  switch (commande) {

    case CMD_ARM: {
      bool on = arg > 0.5f;
      pixhawk_arm(on);
      say(on ? "ARM envoye" : "DISARM envoye");
      break;
    }

    case CMD_MODE: {
      int m = (int)(arg + 0.5f);
      if (m < 1 || m > 6) { say("MODE %d invalide", m); break; }
      pixhawk_set_mode(MODE_TO_ARDU[m]);
      say("MODE %s envoye", MODE_NAME[m]);
      break;
    }

    case CMD_TAKEOFF:
      if (arg <= 0 || arg > MAX_TAKEOFF_M) { say("TAKEOFF refuse (%.0fm)", arg); break; }
      pixhawk_takeoff(arg);
      say("TAKEOFF %.0fm envoye", arg);
      break;

    case CMD_GO: {
      int type = (int)(arg + 0.5f);
      if (type < 1 || type > 7) { say("GO type %d inconnu", type); break; }
      if (type == 1)            { say("GO GPS non gere");           break; }
      if (go_exec(type, arg2)) {
        if (type == 6) say("GO STOP envoye");
        else           say("GO %s %.1f envoye", GO_NAME[type], arg2);
      } else {
        say("GO %s refuse (%.0f)", GO_NAME[type], arg2);
      }
      break;
    }

    default:
      say("Commande %u inconnue", commande);
      break;
  }
}

// ---------------------------------------------------------------
//  A appeler a chaque tour de loop() : renvoie a la base les ACK du Pixhawk
// ---------------------------------------------------------------
static const char* mav_result_txt(uint8_t r) {
  switch (r) {
    case 0:  return "ACCEPTE";
    case 1:  return "refus temp.";
    case 2:  return "REFUSE";
    case 3:  return "non supporte";
    case 4:  return "ECHEC";
    case 5:  return "en cours";
    default: return "?";
  }
}

static const char* mav_cmd_txt(uint16_t c) {
  switch (c) {
    case 400: return "arm";
    case 176: return "mode";
    case 22:  return "takeoff";
    case 115: return "yaw";
    default:  return "cmd";
  }
}

void console_update() {
  uint16_t cmd;
  uint8_t  result;
  if (pixhawk_get_ack(&cmd, &result)) {
    say("ACK %s: %s", mav_cmd_txt(cmd), mav_result_txt(result));
  }
}