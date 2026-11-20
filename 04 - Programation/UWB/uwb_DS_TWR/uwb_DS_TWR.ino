#include <SPI.h>
#include <stdint.h>
#include <string.h>
#include "DW1000.h"
#include "DW1000Time.h"
#include "Arduino.h"

#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define DW_CS     4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

// --- Identity of THIS module : adjust for each board ---
#define SELF_DRONE_ID       0x01   // 0x01 for Drone 1, 0x02 for Drone 2
#define TARGET_PEER_ID      0x02   // 0x02 for Drone 1, 0x01 for Drone 2

#define POLL_INTERVAL_MS    200
#define DISPLAY_INTERVAL_MS 500
#define REPLY_DELAY_US      2500   // Increased delay for ESP32 processing time

#define MSG_TYPE_POLL       0x10
#define MSG_TYPE_RESP       0x20
#define MSG_TYPE_FINAL      0x30

const uint16_t ANTENNA_DELAY_TICKS = 16436; 
const float DISTANCE_PER_TICK      = 0.00469176368f;
const int64_t MASK_40BIT           = 0xFFFFFFFFFFLL;

// 1. Define Kinematics payload first
typedef struct __attribute__((packed)) {
    int16_t pos_x_cm, pos_y_cm, pos_z_cm;
    int16_t vel_x_cms, vel_y_cms, vel_z_cms;
} KinematicsPayload;

// 2. Define UWB Frame payload
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;
    int64_t t_tx_poll;   // Only populated in FINAL message
    int64_t t_rx_resp;   // Only populated in FINAL message
    int64_t t_tx_final;  // Only populated in FINAL message
    KinematicsPayload telemetry;
} UwbFrame;

// --- Global Variables ---
KinematicsPayload my_state;
KinematicsPayload peer_state;

volatile bool rx_packet_ready = false;
volatile bool tx_done = false;

enum TxType { TX_NONE, TX_POLL, TX_RESP, TX_FINAL };
volatile TxType pending_tx_type = TX_NONE;

// State variables for Initiator role (Drone A)
int64_t init_t_tx_poll = 0;
int64_t init_t_rx_resp = 0;
int64_t init_t_tx_final = 0;

// State variables for Responder role (Drone B)
int64_t resp_t_rx_poll = 0;
int64_t resp_t_tx_resp = 0;
int64_t resp_t_rx_final = 0;

uint32_t last_poll_time = 0;
uint32_t last_display_time = 0;

float current_distance_m = -1.0f;
bool peer_connected = false;

// --- Interrupts ---
void IRAM_ATTR handleRxInterrupt() { rx_packet_ready = true; }
void IRAM_ATTR handleTxInterrupt() { tx_done = true; }

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// --- Ranging Functions ---
void sendPoll(uint8_t target_id) {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_POLL;
    f.dest_id = target_id;
    f.src_id = SELF_DRONE_ID;
    f.telemetry = my_state;

    pending_tx_type = TX_POLL;
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void sendResponse(uint8_t target_id) {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_RESP;
    f.dest_id = target_id;
    f.src_id = SELF_DRONE_ID;
    f.telemetry = my_state;

    pending_tx_type = TX_RESP;
    DW1000.newTransmit();
    DW1000.setDefaults();

    // Schedule transmission in the future
    DW1000Time delta(REPLY_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000Time futureTx = DW1000.setDelay(delta);
    resp_t_tx_resp = futureTx.getTimestamp();

    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void sendFinal(uint8_t target_id) {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_FINAL;
    f.dest_id = target_id;
    f.src_id = SELF_DRONE_ID;
    f.telemetry = my_state;

    // Embed Initiator's recorded timestamps into the payload
    f.t_tx_poll = init_t_tx_poll;
    f.t_rx_resp = init_t_rx_resp;

    pending_tx_type = TX_FINAL;
    DW1000.newTransmit();
    DW1000.setDefaults();

    // Schedule transmission in the future
    DW1000Time delta(REPLY_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000Time futureTx = DW1000.setDelay(delta);
    init_t_tx_final = futureTx.getTimestamp();
    
    f.t_tx_final = init_t_tx_final;

    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void processIncomingPacket() {
    UwbFrame f;
    if (DW1000.getDataLength() != sizeof(f)) { 
        receiver(); 
        return; 
    }
    
    DW1000.getData((uint8_t*)&f, sizeof(f));
    
    // Ignore packets not meant for us
    if (f.src_id != TARGET_PEER_ID || f.dest_id != SELF_DRONE_ID) { 
        receiver(); 
        return; 
    }

    // Get the timestamp of the received packet
    DW1000Time rxTime;
    DW1000.getReceiveTimestamp(rxTime);
    int64_t current_rx_ts = rxTime.getTimestamp();

    if (f.msg_type == MSG_TYPE_POLL) {
        resp_t_rx_poll = current_rx_ts;
        peer_state = f.telemetry;
        peer_connected = true;
        sendResponse(f.src_id);
        // Do not call receiver() here; TX is pending
    }
    else if (f.msg_type == MSG_TYPE_RESP) {
        init_t_rx_resp = current_rx_ts;
        peer_state = f.telemetry;
        sendFinal(f.src_id);
        // Do not call receiver() here; TX is pending
    }
    else if (f.msg_type == MSG_TYPE_FINAL) {
        resp_t_rx_final = current_rx_ts;
        peer_state = f.telemetry;
        peer_connected = true;

        // Cast to double immediately to prevent 64-bit integer overflow during multiplication
        double t_round1 = (double)((f.t_rx_resp - f.t_tx_poll) & MASK_40BIT);
        double t_reply1 = (double)((resp_t_tx_resp - resp_t_rx_poll) & MASK_40BIT);
        double t_round2 = (double)((resp_t_rx_final - resp_t_tx_resp) & MASK_40BIT);
        double t_reply2 = (double)((f.t_tx_final - f.t_rx_resp) & MASK_40BIT);

        // Asymmetric DS-TWR formula
        double tof_ticks = (t_round1 * t_round2 - t_reply1 * t_reply2) / (t_round1 + t_round2 + t_reply1 + t_reply2);
        float dist = (float)(tof_ticks * DISTANCE_PER_TICK);

        // Filter out absurd measurements
        if (dist >= 0.0f && dist < 100.0f) {
            current_distance_m = dist; 
        }

        receiver(); // Cycle complete, go back to listening
    }
    else {
        receiver(); // Unknown message type
    }
}

// --- Arduino Setup & Loop ---
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
            init_t_tx_poll = txTime.getTimestamp();
        }
        // TX_RESP and TX_FINAL timestamps are pre-calculated via setDelay(),
        // so we do not need to fetch them from the IC upon TX completion.
        
        pending_tx_type = TX_NONE;
        receiver(); // Always return to listening after sending
    }

    uint32_t now = millis();

    // Only Drone 0x01 initiates the Ranging exchange to prevent RF collisions.
    // Drone 0x02 acts purely as a Responder and will calculate the final distance.
    if (SELF_DRONE_ID == 0x01) {
        if (now - last_poll_time >= POLL_INTERVAL_MS) {
            last_poll_time = now;
            sendPoll(TARGET_PEER_ID);
        }
    }

    // Display output and status
    if (now - last_display_time >= DISPLAY_INTERVAL_MS) {
        last_display_time = now;
        
        if (peer_connected) {
            if (current_distance_m >= 0.0f) {
                // Drone 0x02 will display the distance.
                Serial.printf("Distance : %.2f m\n", current_distance_m);
            } else {
                Serial.println("Peer connected, computing DS-TWR...");
            }
        } else {
            Serial.printf("Searching for peer 0x%02X...\n", TARGET_PEER_ID);
        }
    }
}