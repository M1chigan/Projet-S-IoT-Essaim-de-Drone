#include <SPI.h>
#include <stdint.h>
#include <string.h>
#include "DW1000.h"
#include "DW1000Time.h"
#include "Arduino.h"

// ============================================================================
//  UWB SS-TWR (Single-Sided Two-Way Ranging) — firmware symétrique minimal
//  À flasher tel quel sur les DEUX modules, en changeant uniquement
//  SELF_DRONE_ID / TARGET_PEER_ID ci-dessous.
//  AUCUN print de debug dans le chemin critique : c'est volontaire, un
//  Serial.printf() bloquant entre la réception du POLL et l'appel à
//  setDelay() fausse complètement le timing de la réponse différée.
// ============================================================================

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define DW_CS     4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// --- Identité de CE module : à adapter sur chaque carte ---
#define SELF_DRONE_ID       0x01   // 0x01 sur le drone 1, 0x02 sur le drone 2
#define TARGET_PEER_ID      0x02  // 0x02 sur le drone 1, 0x01 sur le drone 2

#define POLL_INTERVAL_MS    200
#define DISPLAY_INTERVAL_MS 500
#define RESPONSE_DELAY_US   1200

#define MSG_TYPE_POLL       0x10
#define MSG_TYPE_RESP       0x20

const uint16_t ANTENNA_DELAY_TICKS = 16436;   // à calibrer plus tard
const float DISTANCE_PER_TICK      = 0.00469176368f;
const int64_t MASK_40BIT           = 0xFFFFFFFFFFLL;

typedef struct __attribute__((packed)) {
    int16_t pos_x_cm, pos_y_cm, pos_z_cm;
    int16_t vel_x_cms, vel_y_cms, vel_z_cms;
} KinematicsPayload;

typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;
    int64_t poll_rx_ts;
    int64_t resp_tx_ts;
    KinematicsPayload telemetry;
} UwbFrame;

KinematicsPayload my_state;
KinematicsPayload peer_state;

volatile bool rx_packet_ready = false;
volatile bool tx_done = false;

enum TxType { TX_NONE, TX_POLL, TX_RESP };
volatile TxType pending_tx_type = TX_NONE;

int64_t timePollSent = 0;
uint32_t last_poll_time = 0;
uint32_t last_display_time = 0;

float current_distance_m = -1.0f;
bool peer_connected = false;

void IRAM_ATTR handleRxInterrupt() { rx_packet_ready = true; }
void IRAM_ATTR handleTxInterrupt() { tx_done = true; }

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void sendPoll(uint8_t target_id) {
    UwbFrame f;
    f.msg_type = MSG_TYPE_POLL;
    f.dest_id = target_id;
    f.src_id = SELF_DRONE_ID;
    f.poll_rx_ts = 0;
    f.resp_tx_ts = 0;
    f.telemetry = my_state;

    pending_tx_type = TX_POLL;
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void sendResponse(uint8_t target_id, int64_t poll_rx_ts) {
    UwbFrame f;
    f.msg_type = MSG_TYPE_RESP;
    f.dest_id = target_id;
    f.src_id = SELF_DRONE_ID;
    f.poll_rx_ts = poll_rx_ts;
    f.telemetry = my_state;

    pending_tx_type = TX_RESP;
    DW1000.newTransmit();
    DW1000.setDefaults();


    DW1000Time delta(RESPONSE_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000Time futureTx = DW1000.setDelay(delta);
    f.resp_tx_ts = futureTx.getTimestamp();

    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void processIncomingPacket() {
    UwbFrame f;
    if (DW1000.getDataLength() != sizeof(f)) { receiver(); return; }
    
    DW1000.getData((uint8_t*)&f, sizeof(f));
    if (f.src_id != TARGET_PEER_ID || f.dest_id != SELF_DRONE_ID) { receiver(); return; }

    if (f.msg_type == MSG_TYPE_POLL) {
        DW1000Time rxTime;
        DW1000.getReceiveTimestamp(rxTime);
        peer_state = f.telemetry;
        peer_connected = true;
        sendResponse(f.src_id, rxTime.getTimestamp());
        delay(20);
        
    }
    else if (f.msg_type == MSG_TYPE_RESP) {
        DW1000Time rxTime;
        DW1000.getReceiveTimestamp(rxTime);
        int64_t timeRespReceived = rxTime.getTimestamp();

        int64_t round_trip = (timeRespReceived - timePollSent) & MASK_40BIT;
        int64_t reply_time = (f.resp_tx_ts - f.poll_rx_ts) & MASK_40BIT;
        int64_t tof_ticks  = (round_trip - reply_time) / 2;
        float dist = tof_ticks * DISTANCE_PER_TICK;

        if (dist >= 0.0f && dist < 100.0f) current_distance_m = dist*100;

        peer_state = f.telemetry;
        peer_connected = true;

        receiver();
    }

    receiver();
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.printf("=== DRONE 0x%02X ===\n", SELF_DRONE_ID);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    my_state.pos_x_cm = 100;
    my_state.pos_y_cm = -50;
    my_state.pos_z_cm = 150;

    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(SELF_DRONE_ID);
    DW1000.setNetworkId(0xDECA);
    DW1000.enableMode(DW1000.MODE_LONGDATA_FAST_ACCURACY);
    DW1000.setAntennaDelay(ANTENNA_DELAY_TICKS);
    DW1000.commitConfiguration();

    DW1000.attachReceivedHandler(handleRxInterrupt);
    DW1000.attachSentHandler(handleTxInterrupt);

    receiver();
}

void loop() {
    if (rx_packet_ready) {
        rx_packet_ready = false;
        processIncomingPacket();
    }

    if (tx_done) {
        tx_done = false;
        if (pending_tx_type == TX_POLL) {
            DW1000Time txTime;
            DW1000.getTransmitTimestamp(txTime);
            timePollSent = txTime.getTimestamp();
        }
        pending_tx_type = TX_NONE;
        receiver();
    }

    uint32_t now = millis();

    if (now - last_poll_time >= POLL_INTERVAL_MS) {
        last_poll_time = now;
        sendPoll(TARGET_PEER_ID);
    }

    if (now - last_display_time >= DISPLAY_INTERVAL_MS) {
        last_display_time = now;
        if (peer_connected) {
            Serial.printf("Distance : %.2f m\n", current_distance_m);
        } else if (peer_connected) {
            Serial.println("Pair détecté, en attente d'une mesure valide...");
        } else {
            Serial.printf("Recherche du pair 0x%02X...\n", TARGET_PEER_ID);
        }
    }
}