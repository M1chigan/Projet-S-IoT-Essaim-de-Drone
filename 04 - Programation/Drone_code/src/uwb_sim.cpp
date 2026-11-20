#include "config.h"

#if SIM_MODE

#include "uwb_driver.h"
#include "uwb_sim.h"

// Petite file circulaire : les mesures arrivent via MAVLink (dans pixhawk_update),
// et sont consommees par uwb_get_distance() depuis loop(). Meme tache : pas de verrou.
struct UwbMeas { uint8_t id; float d; };
static constexpr uint8_t QSIZE = 16;
static UwbMeas s_q[QSIZE];
static uint8_t s_head = 0, s_tail = 0;

void uwb_sim_push(uint8_t from_id, float dist_m) {
  uint8_t next = (s_head + 1) % QSIZE;
  if (next == s_tail) s_tail = (s_tail + 1) % QSIZE;   // file pleine : on jette la plus ancienne
  s_q[s_head] = { from_id, dist_m };
  s_head = next;
}

bool initUWB()    { return true; }
void uwb_update() {}

bool uwb_get_distance(uint8_t* from_id, float* dist) {
  if (s_tail == s_head) return false;
  *from_id = s_q[s_tail].id;
  *dist    = s_q[s_tail].d;
  s_tail = (s_tail + 1) % QSIZE;
  return true;
}

#endif  // SIM_MODE