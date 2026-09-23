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

// --- Identity of THIS module : adjust for each board (0x01, 0x02, or 0x03) ---
#define SELF_DRONE_ID       0x03  
#define BROADCAST_ID        0xFF
#define MAX_DRONES          5      // Supports IDs up to 0x03

#define POLL_INTERVAL_MS    300    // Base time between polls
#define DISPLAY_INTERVAL_MS 500
#define REPLY_DELAY_US      2500   

#define BASE_DELAY_US       3000 
#define SLOT_DURATION_US    3000

#define MSG_TYPE_POLL       0x10
#define MSG_TYPE_RESP       0x20
#define MSG_TYPE_FINAL      0x30

const uint16_t ANTENNA_DELAY_TICKS = 16436; 
const float DISTANCE_PER_TICK      = 0.00469176368f;
const int64_t MASK_40BIT           = 0xFFFFFFFFFFLL;

// --- Define UWB Frame payload (Stripped down for Ranging ONLY) ---
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;

    int64_t t_tx_poll;   
    int64_t t_rx_resp[MAX_DRONES];   
    int64_t t_tx_final;  
} UwbFrame;

// --- Global Variables ---
volatile bool rx_packet_ready = false;
volatile bool tx_done = false;

enum TxType { TX_NONE, TX_POLL, TX_RESP, TX_FINAL };
volatile TxType pending_tx_type = TX_NONE;

int64_t init_t_tx_poll = 0;
int64_t init_t_tx_final = 0;

int64_t resp_t_rx_poll = 0;
int64_t resp_t_tx_resp = 0;
int64_t resp_t_rx_final = 0;

uint32_t last_poll_time = 0;
uint32_t current_poll_interval = POLL_INTERVAL_MS;
uint32_t last_display_time = 0;

int64_t init_t_rx_resp_array[MAX_DRONES] = {0};
float current_distances_m[MAX_DRONES] = {-1.0f, -1.0f, -1.0f, -1.0f}; // Array for multiple peers

bool waiting_for_responses = false;
uint32_t poll_sent_micros = 0;
const uint32_t FINAL_TRIGGER_DELAY_US = 12000;

// --- Interrupts ---
void IRAM_ATTR handleRxInterrupt() { rx_packet_ready = true; }
void IRAM_ATTR handleTxInterrupt() { tx_done = true; }

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

// Dynamically assign slots (0, 1, 2...) skipping the drone that sent the POLL
uint32_t getResponderDelayUs(uint8_t my_id, uint8_t initiator_id) {
    uint8_t slot_index = 0;
    for (uint8_t i = 1; i < MAX_DRONES; i++) {
        if (i == initiator_id) continue;
        if (i == my_id) break;
        slot_index++;
    }
    return BASE_DELAY_US + (slot_index * SLOT_DURATION_US);
}

// --- Ranging Functions ---
void sendPoll() {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_POLL;
    f.dest_id = BROADCAST_ID;
    f.src_id = SELF_DRONE_ID;

    pending_tx_type = TX_POLL;
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void sendResponse(uint8_t target_id, uint32_t delay_us) {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_RESP;
    f.dest_id = target_id; 
    f.src_id = SELF_DRONE_ID;
    
    pending_tx_type = TX_RESP;
    DW1000.newTransmit();
    DW1000.setDefaults();

    DW1000Time delta(delay_us, DW1000Time::MICROSECONDS);
    DW1000Time futureTx = DW1000.setDelay(delta);
    resp_t_tx_resp = futureTx.getTimestamp();

    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void sendFinal() {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_FINAL;
    f.dest_id = BROADCAST_ID;
    f.src_id = SELF_DRONE_ID;

    f.t_tx_poll = init_t_tx_poll;

    for(int i = 0; i < MAX_DRONES; i++) {
        f.t_rx_resp[i] = init_t_rx_resp_array[i];
    }

    pending_tx_type = TX_FINAL;
    DW1000.newTransmit();
    DW1000.setDefaults();

    DW1000Time delta(REPLY_DELAY_US, DW1000Time::MICROSECONDS);
    DW1000Time futureTx = DW1000.setDelay(delta);
    init_t_tx_final = futureTx.getTimestamp();
    
    f.t_tx_final = init_t_tx_final;

    DW1000.setData((uint8_t*)&f, sizeof(f));
    DW1000.startTransmit();
}

void processIncomingPacket() {
    UwbFrame f;
    if (DW1000.getDataLength() != sizeof(f)) { receiver(); return; }
    DW1000.getData((uint8_t*)&f, sizeof(f));
    
    // Accept messages meant for me OR broadcast messages
    if (f.dest_id != SELF_DRONE_ID && f.dest_id != BROADCAST_ID) { 
        receiver(); return; 
    }

    DW1000Time rxTime;
    DW1000.getReceiveTimestamp(rxTime);
    int64_t current_rx_ts = rxTime.getTimestamp();

    if (f.msg_type == MSG_TYPE_POLL) {
        resp_t_rx_poll = current_rx_ts;
        
        // Calculate delay based on who initiated the POLL
        sendResponse(f.src_id, getResponderDelayUs(SELF_DRONE_ID, f.src_id));
    }
    
    else if (f.msg_type == MSG_TYPE_RESP) {
        // Only record if I am currently the Initiator waiting for responses
        if (waiting_for_responses && f.dest_id == SELF_DRONE_ID) {
            if (f.src_id < MAX_DRONES) {
                init_t_rx_resp_array[f.src_id] = current_rx_ts;
            }
        }
        receiver(); 
    }
    
    else if (f.msg_type == MSG_TYPE_FINAL) {
        resp_t_rx_final = current_rx_ts;
        
        // The Initiator has embedded my specific RX timestamp in its array
        int64_t my_t_rx_resp = f.t_rx_resp[SELF_DRONE_ID];
        
        if (my_t_rx_resp != 0) {
            double t_round1 = (double)((my_t_rx_resp - f.t_tx_poll) & MASK_40BIT);
            double t_reply1 = (double)((resp_t_tx_resp - resp_t_rx_poll) & MASK_40BIT);
            double t_round2 = (double)((resp_t_rx_final - resp_t_tx_resp) & MASK_40BIT);
            double t_reply2 = (double)((f.t_tx_final - my_t_rx_resp) & MASK_40BIT);

            double tof_ticks = (t_round1 * t_round2 - t_reply1 * t_reply2) / (t_round1 + t_round2 + t_reply1 + t_reply2);
            float dist = (float)(tof_ticks * DISTANCE_PER_TICK);
            
            // Save the computed distance to the specific Drone that initiated
            if (dist >= 0.0f && dist < 100.0f) {
                current_distances_m[f.src_id] = dist; 
            }
        }
        receiver();
    }
}

// --- Arduino Setup & Loop ---
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.printf("=== DRONE 0x%02X ===\n", SELF_DRONE_ID);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

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
            
            // CRITICAL: Start the response window timer only when POLL physically leaves antenna
            poll_sent_micros = micros();
            waiting_for_responses = true;
            memset(init_t_rx_resp_array, 0, sizeof(init_t_rx_resp_array)); // Clear old data
        }
        
        pending_tx_type = TX_NONE;
        receiver(); 
    }

    uint32_t now_ms = millis();
    uint32_t now_us = micros();

    // EVERY drone acts as an initiator with a randomized jitter to avoid permanent collisions
    if (now_ms - last_poll_time >= current_poll_interval) {
        last_poll_time = now_ms;
        current_poll_interval = POLL_INTERVAL_MS + random(10, 80); // Organic TDMA shift
        
        if (!waiting_for_responses) { // Only send POLL if we are not busy listening
            sendPoll();
        }
    }
    
    // Close the listening window and shoot the FINAL message
    if (waiting_for_responses && (now_us - poll_sent_micros >= FINAL_TRIGGER_DELAY_US)) {
        waiting_for_responses = false;
        sendFinal();
    }

    // Display output and status
    if (now_ms - last_display_time >= DISPLAY_INTERVAL_MS) {
        last_display_time = now_ms;
        bool has_connections = false;
        
        Serial.printf("\n--- Status Drone 0x%02X ---\n", SELF_DRONE_ID);
        for (int i = 1; i < MAX_DRONES; i++) {
            if (i != SELF_DRONE_ID && current_distances_m[i] >= 0.0f) {
                Serial.printf("Distance to 0x%02X : %.2f cm\n", i, current_distances_m[i]*100);
                has_connections = true;
            }
        }
        
        if (!has_connections) {
            Serial.println("Searching for peers...");
        }
    }
}