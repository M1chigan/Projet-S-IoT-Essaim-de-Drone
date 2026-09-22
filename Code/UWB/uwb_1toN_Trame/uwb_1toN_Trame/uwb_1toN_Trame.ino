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
#define SELF_DRONE_ID       0x01   
#define BROADCAST_ID        0xFF
#define MAX_DRONES          5      

#define POLL_INTERVAL_MS    300    
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

// --- Define UWB Frame payload (Optimized for cm) ---
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;

    // DS-TWR Timestamps
    int64_t t_tx_poll;   
    int64_t t_rx_resp[MAX_DRONES];   
    int64_t t_tx_final;  
    
    // --- CUSTOM PAYLOAD ---
    // Distances in centimeters using int16_t (Max ~327 meters)
    int16_t distances_cm[MAX_DRONES];

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

// The Swarm Mental Map: Distances in cm (Values < 0 mean unknown)
int16_t network_distances_cm[MAX_DRONES][MAX_DRONES];

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

uint32_t getResponderDelayUs(uint8_t my_id, uint8_t initiator_id) {
    uint8_t slot_index = 0;
    for (uint8_t i = 1; i < MAX_DRONES; i++) {
        if (i == initiator_id) continue;
        if (i == my_id) break;
        slot_index++;
    }
    return BASE_DELAY_US + (slot_index * SLOT_DURATION_US);
}

// Helper function to inject local distances into outgoing frame
void populatePayload(UwbFrame *f) {
    for (int i = 0; i < MAX_DRONES; i++) {
        f->distances_cm[i] = network_distances_cm[SELF_DRONE_ID][i];
    }
}

// --- Ranging Functions ---
void sendPoll() {
    UwbFrame f;
    memset(&f, 0, sizeof(f));
    f.msg_type = MSG_TYPE_POLL;
    f.dest_id = BROADCAST_ID;
    f.src_id = SELF_DRONE_ID;
    
    populatePayload(&f); 

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
    
    populatePayload(&f);
    
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
    
    populatePayload(&f); 

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
    
    if (f.dest_id != SELF_DRONE_ID && f.dest_id != BROADCAST_ID) { 
        receiver(); return; 
    }

    DW1000Time rxTime;
    DW1000.getReceiveTimestamp(rxTime);
    int64_t current_rx_ts = rxTime.getTimestamp();
    
    // --- GOSSIP PROTOCOL UPDATE WITH BOUNDS CHECK ---
    // Safety check: Prevent memory corruption if src_id is malformed
    if (f.src_id < MAX_DRONES) {
        for (int i = 0; i < MAX_DRONES; i++) {
            if (f.distances_cm[i] >= 0) {
                network_distances_cm[f.src_id][i] = f.distances_cm[i];
            }
        }
    }

    if (f.msg_type == MSG_TYPE_POLL) {
        resp_t_rx_poll = current_rx_ts;
        sendResponse(f.src_id, getResponderDelayUs(SELF_DRONE_ID, f.src_id));
    }
    
    else if (f.msg_type == MSG_TYPE_RESP) {
        if (waiting_for_responses && f.dest_id == SELF_DRONE_ID) {
            if (f.src_id < MAX_DRONES) {
                init_t_rx_resp_array[f.src_id] = current_rx_ts;
            }
        }
        receiver(); 
    }
    
    else if (f.msg_type == MSG_TYPE_FINAL) {
        resp_t_rx_final = current_rx_ts;
        int64_t my_t_rx_resp = f.t_rx_resp[SELF_DRONE_ID];
        
        if (my_t_rx_resp != 0) {
            double t_round1 = (double)((my_t_rx_resp - f.t_tx_poll) & MASK_40BIT);
            double t_reply1 = (double)((resp_t_tx_resp - resp_t_rx_poll) & MASK_40BIT);
            double t_round2 = (double)((resp_t_rx_final - resp_t_tx_resp) & MASK_40BIT);
            double t_reply2 = (double)((f.t_tx_final - my_t_rx_resp) & MASK_40BIT);

            // Prevent division by zero just in case
            double denominator = t_round1 + t_round2 + t_reply1 + t_reply2;
            
            if (denominator != 0) {
                double tof_ticks = (t_round1 * t_round2 - t_reply1 * t_reply2) / denominator;
                float dist_m = (float)(tof_ticks * DISTANCE_PER_TICK);
                
                // If distance is plausible (between 0 and 100 meters)
                if (dist_m >= 0.0f && dist_m < 100.0f) {
                    
                    // Convert to centimeters
                    int16_t dist_cm = (int16_t)(dist_m * 100.0f);
                    
                    // CRITICAL FIX: Ensure array bounds are respected to prevent ESP32 crashes
                    if (f.src_id < MAX_DRONES) {
                        network_distances_cm[SELF_DRONE_ID][f.src_id] = dist_cm; 
                        network_distances_cm[f.src_id][SELF_DRONE_ID] = dist_cm; 
                    }
                }
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

    // Initialize all distances to -1 (unknown)
    for(int i = 0; i < MAX_DRONES; i++) {
        for(int j = 0; j < MAX_DRONES; j++) {
            network_distances_cm[i][j] = -1;
        }
    }

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

    // --- LA CORRECTION ANTI-CRASH ---
    // La librairie vient d'attacher une interruption, on la détache immédiatement
    // pour éviter que l'ESP32 ne fasse du SPI en arrière-plan et ne crashe.
    detachInterrupt(digitalPinToInterrupt(PIN_IRQ));

    receiver();
}

void loop() {
    // --- GESTION MANUELLE DE LA PUCE UWB ---
    // Au lieu de crasher dans une interruption, on lit la broche ici en toute sécurité
    if (digitalRead(PIN_IRQ) == HIGH) {
        DW1000.handleInterrupt();
    }

    // --- LE RESTE DU CODE RESTE IDENTIQUE ---
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
            
            poll_sent_micros = micros();
            waiting_for_responses = true;
            memset(init_t_rx_resp_array, 0, sizeof(init_t_rx_resp_array)); 
        }
        
        pending_tx_type = TX_NONE;
        receiver(); 
    }

    uint32_t now_ms = millis();
    uint32_t now_us = micros();

    if (now_ms - last_poll_time >= current_poll_interval) {
        last_poll_time = now_ms;
        current_poll_interval = POLL_INTERVAL_MS + random(10, 80); 
        
        if (!waiting_for_responses) { 
            sendPoll();
        }
    }
    
    if (waiting_for_responses && (now_us - poll_sent_micros >= FINAL_TRIGGER_DELAY_US)) {
        waiting_for_responses = false;
        sendFinal();
    }

    // --- AFFICHAGE ---
    if (now_ms - last_display_time >= DISPLAY_INTERVAL_MS) {
        last_display_time = now_ms;
        bool has_connections = false;
        
        Serial.printf("\n--- SWARM MAP (Drone 0x%02X) ---\n", SELF_DRONE_ID);
        
        for (int i = 1; i < MAX_DRONES; i++) {
            if (i != SELF_DRONE_ID && network_distances_cm[SELF_DRONE_ID][i] >= 0) {
                Serial.printf("[LOCAL] Distance to Drone 0x%02X : %d cm\n", i, network_distances_cm[SELF_DRONE_ID][i]);
                has_connections = true;
            }
        }
        
        for (int i = 1; i < MAX_DRONES; i++) {
            for (int j = i + 1; j < MAX_DRONES; j++) {
                if (i == SELF_DRONE_ID || j == SELF_DRONE_ID) continue;
                
                if (network_distances_cm[i][j] >= 0) {
                    Serial.printf("[GOSSIP] Drone 0x%02X to Drone 0x%02X : %d cm\n", i, j, network_distances_cm[i][j]);
                }
            }
        }
        
        if (!has_connections) {
            Serial.println("Searching for peers...");
        }
    }
}