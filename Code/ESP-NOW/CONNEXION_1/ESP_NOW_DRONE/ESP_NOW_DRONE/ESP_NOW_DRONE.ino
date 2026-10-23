#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

// ---------- Réglages ----------
const uint8_t  CHANNEL   = 1;      // identique sur tous les ESP32
const uint8_t  FIRST_ID  = 1;      // id du premier drone simulé
const uint8_t  NB_DRONES = 3;      // 3, 5... pour simuler un essaim
const uint32_t PERIOD_MS = 20;     // 50 Hz

// Point de départ commun de l'essaim (Strasbourg, à adapter)
const double HOME_LAT   = 48.5839;
const double HOME_LON   = 7.7455;
const float  HOME_ALT_M = 142.0f;  // altitude du sol au-dessus du niveau de la mer
// ------------------------------

uint8_t BASE_MAC[6] = {0xCC, 0xDB, 0xA7, 0x07, 0x15, 0x50};

// ---------- Trame (structs identiques à celles de la base) ----------
struct Attitude {
  float roll;
  float pitch;
  float yaw;
};

struct GPS {
  int32_t lat_e7;      // latitude  en degrés x 10^7
  int32_t lon_e7;      // longitude en degrés x 10^7
  int32_t alt_mm;      // altitude en mm (au-dessus du niveau de la mer)
  uint8_t fix_type;    // 0-1 = pas de fix, 2 = 2D, 3 = 3D
  uint8_t satellites;  // nombre de satellites visibles
};

struct Altitude {
  float amsl_m;
};

typedef struct __attribute__((packed)) {
  uint8_t  id;
  uint32_t t_ms;
  float d;
  float z;
  GPS gps;
  Attitude a;
} Pkt;

// GPS n'est pas packed : le compilateur ajoute 2 octets de padding (14 -> 16).
// La base doit utiliser exactement les mêmes définitions.
static_assert(sizeof(Pkt) == 1 + 4 + 4 + 4 + sizeof(GPS) + sizeof(Attitude),
              "Taille de trame inattendue");
static_assert(sizeof(Pkt) <= 250, "Trame trop grande pour ESP-NOW");

// ---------- Paramètres propres à chaque drone ----------
struct DroneParams {
  float cn, ce;          // centre de l'orbite par rapport à HOME (m, nord/est)
  float radius;          // rayon de l'orbite (m)
  float omega;           // vitesse angulaire (rad/s), signe = sens de rotation
  float phase;           // déphasage initial
  float altBase;         // altitude relative moyenne (m)
  float altAmp;          // amplitude de variation d'altitude (m)
  float altFreq;         // fréquence de variation d'altitude (rad/s)
  float pitchAmp;        // amplitude du tangage (deg)
  float gpsNoise;        // bruit de position GPS (m)
  uint8_t sats;          // nombre de satellites moyen
};

DroneParams params[NB_DRONES];

// Tirage uniforme aléatoire (différent à chaque démarrage)
float rnd(float a, float b) {
  return a + (b - a) * (esp_random() / 4294967295.0f);
}

// Bruit gaussien approximatif (somme de 4 tirages uniformes)
float noise(float amp) {
  float s = 0;
  for (int i = 0; i < 4; i++) s += rnd(-1.0f, 1.0f);
  return amp * s / 4.0f;
}

void initParams() {
  for (uint8_t k = 0; k < NB_DRONES; k++) {
    DroneParams &p = params[k];
    p.cn       = rnd(-30, 30);
    p.ce       = rnd(-30, 30);
    p.radius   = rnd(5, 25);
    p.omega    = rnd(0.15f, 0.6f) * ((esp_random() & 1) ? 1 : -1);
    p.phase    = rnd(0, 2 * PI);
    p.altBase  = rnd(8, 40);
    p.altAmp   = rnd(0.5f, 3.0f);
    p.altFreq  = rnd(0.2f, 1.0f);
    p.pitchAmp = rnd(2, 8);
    p.gpsNoise = rnd(0.1f, 0.6f);
    p.sats     = (uint8_t)rnd(7, 15);
  }
}

// ---------- Simulation ----------
Pkt simulate(uint8_t k) {
  const DroneParams &p = params[k];
  float t  = millis() / 1000.0f;
  float th = p.omega * t + p.phase;      // angle sur l'orbite

  // Position locale (m) : orbite circulaire autour du centre propre au drone
  float n = p.cn + p.radius * sinf(th);
  float e = p.ce + p.radius * cosf(th);

  // Vitesse (m/s)
  float vn = p.radius * p.omega * cosf(th);
  float ve = -p.radius * p.omega * sinf(th);
  float v  = fabsf(p.radius * p.omega);

  // Altitude relative au point de départ
  float z = p.altBase + p.altAmp * sinf(p.altFreq * t + p.phase) + noise(0.05f);

  // Position GPS (avec bruit) convertie en degrés
  float nn = n + noise(p.gpsNoise);
  float ee = e + noise(p.gpsNoise);
  double dLat = nn / 111320.0;
  double dLon = ee / (111320.0 * cos(HOME_LAT * DEG_TO_RAD));

  // Satellites : varie légèrement
  int sats = p.sats + (int)roundf(noise(1.5f));
  if (sats < 4) sats = 4;

  // Attitude : yaw = direction de déplacement, roll = inclinaison du virage
  float yaw = atan2f(ve, vn) * RAD_TO_DEG;
  if (yaw < 0) yaw += 360.0f;
  float roll = atan2f(v * p.omega, 9.81f) * RAD_TO_DEG;

  Pkt pkt;
  memset(&pkt, 0, sizeof(pkt));          // met le padding à 0
  pkt.id   = FIRST_ID + k;
  pkt.t_ms = millis();
  pkt.d    = sqrtf(n * n + e * e) + noise(0.05f);
  pkt.z    = z;

  pkt.gps.lat_e7     = (int32_t)llround((HOME_LAT + dLat) * 1e7);
  pkt.gps.lon_e7     = (int32_t)llround((HOME_LON + dLon) * 1e7);
  pkt.gps.alt_mm     = (int32_t)lroundf((HOME_ALT_M + z + noise(0.3f)) * 1000.0f);
  pkt.gps.fix_type   = 3;
  pkt.gps.satellites = (uint8_t)sats;

  pkt.a.roll  = roll + noise(0.8f);
  pkt.a.pitch = p.pitchAmp * sinf(0.6f * t + p.phase) + noise(0.5f);
  pkt.a.yaw   = yaw + noise(1.0f);
  return pkt;
}

// ---------- Réception des commandes de la base ----------
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  Serial.printf("Commande recue (%d octets) :", len);
  for (int i = 0; i < len; i++) Serial.printf(" %u", data[i]);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW KO");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BASE_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onRecv);

  initParams();

  Serial.printf("Simulateur pret : %d drone(s), ids %d a %d, trame de %u octets\n",
                NB_DRONES, FIRST_ID, FIRST_ID + NB_DRONES - 1, (unsigned)sizeof(Pkt));
  for (uint8_t k = 0; k < NB_DRONES; k++) {
    Serial.printf("  Drone %d : rayon %.1f m, omega %.2f rad/s, alt %.1f m, %d sats\n",
                  FIRST_ID + k, params[k].radius, params[k].omega,
                  params[k].altBase, params[k].sats);
  }
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= PERIOD_MS) {
    last = millis();
    for (uint8_t k = 0; k < NB_DRONES; k++) {
      Pkt p = simulate(k);
      esp_now_send(BASE_MAC, (uint8_t*)&p, sizeof(p));
      delay(2);
    }
  }
}