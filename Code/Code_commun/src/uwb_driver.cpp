
#include "uwb_driver.h"
#include "config.h"


#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#include "DW1000.h"
#include "DW1000Time.h"




// Antenna et module config :
static constexpr uint16_t NETWORK_ID          = 0xDECA;
static constexpr uint16_t ANTENNA_DELAY_TICKS = 16436;
static constexpr double   DISTANCE_PER_TICK   = 0.00469176368; 
static constexpr int64_t  MASK_40BIT          = 0xFFFFFFFFFFLL; 

// Types de messages & Broadcast :
static constexpr uint8_t BROADCAST_ID = 0xFF;
static constexpr uint8_t MSG_POLL     = 0x10;
static constexpr uint8_t MSG_RESP     = 0x20;
static constexpr uint8_t MSG_FINAL    = 0x30;

// Timing (en microsecondes)
static constexpr uint32_t BASE_DELAY_US    = 3000;  // délai de réponse du 1er répondeur
static constexpr uint32_t SLOT_DURATION_US = 3000;  // écart entre deux répondeurs
static constexpr uint32_t FINAL_DELAY_US   = 2500;  // délai d'émission du FINAL
// Fenêtre pendant laquelle l'initiateur écoute les réponses (+1 slot de marge)
static constexpr uint32_t RESP_WINDOW_US   = BASE_DELAY_US + (UWB_MAX_DRONES - 1) * SLOT_DURATION_US;
static constexpr uint32_t TX_TIMEOUT_US    = 20000; // une émission doit finir avant
static constexpr uint32_t FINAL_TIMEOUT_US = RESP_WINDOW_US + FINAL_DELAY_US + 5000;

//Trame pour la distance en UWB 
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;
    int64_t t_tx_poll;                    
    int64_t t_rx_resp[UWB_MAX_DRONES];    
    int64_t t_tx_final;                   
} UwbFrame;


// Etat de l'UWB 
enum class State : uint8_t {
    IDLE,
    POLL_TX,     // initiateur : poll en cours d'émission
    WAIT_RESP,   // initiateur : on collecte les réponses
    FINAL_TX,    // initiateur : final en cours d'émission
    RESP_TX,     // répondeur  : réponse en cours d'émission
    WAIT_FINAL   // répondeur  : on attend le final
};

static State    s_state = State::IDLE;
static uint32_t s_state_t0 = 0;  

static uint8_t s_self_id  = 0;
static uint8_t s_self_idx = 0; //indice dans les tableaux = id - 1

//Variales liés au interruptions
static volatile bool s_rx_ready    = false;
static volatile bool s_tx_done     = false;
static volatile bool s_radio_error = false;

//Données si INITIATEUR
static int64_t s_t_tx_poll = 0;
static int64_t s_t_rx_resp[UWB_MAX_DRONES] = {0};
static uint8_t s_n_resp = 0;

//Données si RÉPONDEUR
static uint8_t s_peer_id    = 0;
static int64_t s_t_rx_poll  = 0;
static int64_t s_t_tx_resp  = 0;

//Stockage des distances et disponibilité
static bool    s_new = false;
static uint8_t s_new_from = 0;
static float   s_new_dist = 0.0f;

// Variables d'ordonancement des POLL
static uint8_t  s_last_initiator = 0; 
static uint32_t s_last_activity_ms = 0;

static uint32_t PERIOD_MS   = 1000; 

constexpr uint32_t TURN_GAP_MS        = 20;   
constexpr uint32_t TAKEOVER_MS        = 300;  
constexpr uint32_t TAKEOVER_STAGGER_MS = 50;

static inline bool    isValidId(uint8_t id) { return id >= 1 && id <= UWB_MAX_DRONES; }
static inline uint8_t idxOf(uint8_t id)     { return id - 1; }

//Gestion INTERUPTIONS : 
static void onRxInterrupt()    { s_rx_ready = true; }
static void onTxInterrupt()    { s_tx_done = true; }
static void onErrorInterrupt() { s_radio_error = true; }


static void setState(State s) {
    s_state = s;
    s_state_t0 = micros();
}

static inline double diff40(int64_t a, int64_t b) {
    return (double)((a - b) & MASK_40BIT);
}

static void startReceiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

static void resetToIdle() {
    setState(State::IDLE);
    startReceiver();
}

static UwbFrame makeFrame(uint8_t type, uint8_t dest) {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = type;
    f.dest_id  = dest;
    f.src_id   = s_self_id;
    return f;
}

static int64_t transmit(UwbFrame& f, uint32_t delay_us) {
    int64_t scheduled = 0;
 
    DW1000.newTransmit();
    DW1000.setDefaults();
 
    if (delay_us > 0) {
        DW1000Time delta(static_cast<int32_t>(delay_us), DW1000Time::MICROSECONDS);
        scheduled = DW1000.setDelay(delta).getTimestamp();
        if (f.msg_type == MSG_FINAL) f.t_tx_final = scheduled; 
    }
 
    DW1000.setData(reinterpret_cast<uint8_t*>(&f), sizeof(f));
    DW1000.startTransmit();
    return scheduled;
}
static uint32_t responderDelayUs(uint8_t initiator_id) {
    uint8_t slot = s_self_idx;
    if (s_self_idx > idxOf(initiator_id)) slot--;
    return BASE_DELAY_US + slot * SLOT_DURATION_US;
}

static void markActivity(uint8_t initiator_id) {
    s_last_initiator   = initiator_id;
    s_last_activity_ms = millis();
}


// =====================================================
// Fonctions liés au protocole 
// ====================================================

// Initiateur : envoie le FINAL avec tous les timestamps collectés
static void sendFinal() {
    UwbFrame f = makeFrame(MSG_FINAL, BROADCAST_ID);
    f.t_tx_poll = s_t_tx_poll;
    for (uint8_t i = 0; i < UWB_MAX_DRONES; i++) f.t_rx_resp[i] = s_t_rx_resp[i];
 
    setState(State::FINAL_TX);
    transmit(f, FINAL_DELAY_US);
}

// Répondeur : reçoit un POLL. Retourne true si un RESP  a été lancée.
static bool handlePoll(const UwbFrame& f, int64_t rx_ts) {
    if (s_state != State::IDLE) return false;   // déjà occupé : on ignore
 
    s_peer_id   = f.src_id;
    s_t_rx_poll = rx_ts;
 
    UwbFrame r = makeFrame(MSG_RESP, f.src_id);
    setState(State::RESP_TX);
    s_t_tx_resp = transmit(r, responderDelayUs(f.src_id));
    return true;
}

// Initiateur : reçoit une TRAME 
static void handleResp(const UwbFrame& f, int64_t rx_ts) {
    if (s_state != State::WAIT_RESP || f.dest_id != s_self_id) return;
 
    uint8_t i = idxOf(f.src_id);
    if (s_t_rx_resp[i] == 0) s_n_resp++;
    s_t_rx_resp[i] = rx_ts;
}

// Répondeur : reçoit le FINAL, calcule la distance (DS-TWR)
static void handleFinal(const UwbFrame& f, int64_t rx_ts) {
    if (s_state != State::WAIT_FINAL || f.src_id != s_peer_id) return;
 
    int64_t my_rx_resp = f.t_rx_resp[s_self_idx];
    if (my_rx_resp != 0) {
        double round1 = diff40(my_rx_resp, f.t_tx_poll);
        double reply1 = diff40(s_t_tx_resp, s_t_rx_poll);
        double round2 = diff40(rx_ts, s_t_tx_resp);
        double reply2 = diff40(f.t_tx_final, my_rx_resp);
        
        double tof  = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
        float  dist = (float)(tof * DISTANCE_PER_TICK);
        
        Serial.printf("r1=%.0f p1=%.0f r2=%.0f p2=%.0f tof=%.1f dist=%.3f\n",
              round1, reply1, round2, reply2, tof, dist);

        if (dist > -5.0f && dist < 100.0f) {
            s_new_from = f.src_id;
            s_new_dist = (dist < 0.0f) ? 0.0f : dist;
            s_new = true;
            
        }
    }
    setState(State::IDLE);
}

// Traite une trame reçue
static void handleRx() {
    bool tx_started = false;
    UwbFrame f;
 
    if (DW1000.getDataLength() == sizeof(f)) {
        DW1000.getData(reinterpret_cast<uint8_t*>(&f), sizeof(f));
 
        DW1000Time rx;
        DW1000.getReceiveTimestamp(rx);
        int64_t rx_ts = rx.getTimestamp();
 
        bool for_me = (f.dest_id == s_self_id || f.dest_id == BROADCAST_ID);
        if (for_me && isValidId(f.src_id) && f.src_id != s_self_id) {
            if (f.msg_type == MSG_POLL || f.msg_type == MSG_FINAL) markActivity(f.src_id);
            switch (f.msg_type) {
                case MSG_POLL:  tx_started = handlePoll(f, rx_ts); break;
                case MSG_RESP:  handleResp(f, rx_ts);              break;
                case MSG_FINAL: handleFinal(f, rx_ts);             break;
                default: break;
            }
        }
    }
    if (!tx_started) startReceiver();
}

// Une émission vient de se terminer
static void handleTxDone() {
    switch (s_state) {
        case State::POLL_TX: {
            DW1000Time tx;
            DW1000.getTransmitTimestamp(tx);
            s_t_tx_poll = tx.getTimestamp();
            setState(State::WAIT_RESP);
            break;
        }
        case State::FINAL_TX:
            markActivity(s_self_id);
            setState(State::IDLE);
            break;
        case State::RESP_TX:
            setState(State::WAIT_FINAL);
            break;
        default:
            break;
    }
    startReceiver();
}

static void checkTimeouts() {
    uint32_t elapsed = micros() - s_state_t0;   
 
    switch (s_state) {
        case State::WAIT_RESP:
            if (elapsed >= RESP_WINDOW_US) {
                if (s_n_resp > 0) sendFinal();
                else              resetToIdle();
            }
            break;
        case State::POLL_TX:
        case State::FINAL_TX:
        case State::RESP_TX:
            if (elapsed >= TX_TIMEOUT_US) resetToIdle();
            break;
        case State::WAIT_FINAL:
            if (elapsed >= FINAL_TIMEOUT_US) resetToIdle();
            break;
        default:
            break;
    }
}

bool initUWB() {
    if (!isValidId(DRONE_ID)) return false;
    s_self_id  = DRONE_ID;
    s_self_idx = idxOf(DRONE_ID);
 
    SPI.begin(UWB_PIN_SCK, UWB_PIN_MISO, UWB_PIN_MOSI);
    DW1000.begin(UWB_PIN_IRQ, UWB_PIN_RST);
    DW1000.select(UWB_PIN_SS);
 
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(DRONE_ID);
    DW1000.setNetworkId(NETWORK_ID);
    DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);
    DW1000.setAntennaDelay(ANTENNA_DELAY_TICKS);
    DW1000.commitConfiguration();
 
    DW1000.attachReceivedHandler(onRxInterrupt);
    DW1000.attachSentHandler(onTxInterrupt);
    DW1000.attachReceiveFailedHandler(onErrorInterrupt);
    DW1000.attachReceiveTimeoutHandler(onErrorInterrupt);
    DW1000.attachErrorHandler(onErrorInterrupt);
 
    setState(State::IDLE);
    startReceiver();
    return true;
}

void uwb_update() {
    if (s_tx_done)     { s_tx_done = false;     handleTxDone(); }
    if (s_rx_ready)    { s_rx_ready = false;    handleRx(); }
    if (s_radio_error) { s_radio_error = false; startReceiver(); }
    checkTimeouts();
}

bool uwb_start_ranging() {
    if (s_state != State::IDLE) return false;
 
    memset(s_t_rx_resp, 0, sizeof(s_t_rx_resp));
    s_n_resp = 0;

    markActivity(s_self_id);
    UwbFrame f = makeFrame(MSG_POLL, BROADCAST_ID);
    setState(State::POLL_TX);
    transmit(f, 0);
    return true;
}

bool uwb_get_distance(uint8_t* from_id, float* d) {
    if (!s_new) return false;
    s_new = false;
    *from_id = s_new_from;
    *d = s_new_dist;
    return true;
}

bool uwb_is_idle() {
    return s_state == State::IDLE;
}
 
uint8_t uwb_last_initiator() {
    return s_last_initiator;
}
 
uint32_t uwb_ms_since_activity() {
    return millis() - s_last_activity_ms;
}

void scheduler_update() {
    if (!uwb_is_idle()) return;
 
    uint32_t quiet = uwb_ms_since_activity();
    uint8_t next = uwb_last_initiator() % UWB_MAX_DRONES + 1;  
 
    bool my_turn  = (next == DRONE_ID) && (quiet >= TURN_GAP_MS);
    bool takeover = (quiet >= TAKEOVER_MS + DRONE_ID * TAKEOVER_STAGGER_MS);
 
    if (my_turn || takeover) uwb_start_ranging();
}





