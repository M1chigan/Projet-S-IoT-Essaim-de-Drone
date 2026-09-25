#include <SPI.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
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

// --- Kalman Filter ---
struct KalmanTrack {
    float x;                // Estimated distance (m)
    float v;                // Estimated velocity (m/s)
    float p00, p01, p11;    // Error covariance
    uint32_t last_us;
    bool initialized;
};

const float KALMAN_Q = 0.05f;     // Drone acceleration variance (m^2/s^3)
const float KALMAN_R = 0.02f;   // UWB distance variance (~7 cm jitter)

KalmanTrack kalman_filters[MAX_DRONES];

void initKalmanTrack(KalmanTrack &track) {
    track.x = 0.0f;
    track.v = 0.0f;
    track.p00 = 1.0f;
    track.p01 = 0.0f;
    track.p11 = 1.0f;
    track.last_us = 0;
    track.initialized = false;
}

float updateKalmanTrack(KalmanTrack &track, float raw_dist_m) {
    uint32_t now_us = micros();

    if (!track.initialized) {
        track.x = raw_dist_m;
        track.v = 0.0f;
        track.last_us = now_us;
        track.initialized = true;
        return raw_dist_m;
    }

    float dt = (now_us - track.last_us) * 1e-6f;
    track.last_us = now_us;

    if (dt <= 0.0f || dt > 1.0f) {
        dt = 0.02f;
    }

    // 1. Predict
    track.x += track.v * dt;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    track.p00 += dt * (2.0f * track.p01 + dt * track.p11) + KALMAN_Q * (dt3 / 3.0f);
    track.p01 += dt * track.p11 + KALMAN_Q * (dt2 / 2.0f);
    track.p11 += KALMAN_Q * dt;

    // 2. Update
    float y = raw_dist_m - track.x;
    float s = track.p00 + KALMAN_R;
    float k0 = track.p00 / s;
    float k1 = track.p01 / s;

    track.x += k0 * y;
    track.v += k1 * y;

    float p00_prev = track.p00;
    float p01_prev = track.p01;

    track.p00 -= k0 * p00_prev;
    track.p01 -= k0 * p01_prev;
    track.p11 -= k1 * p01_prev;

    return track.x;
}

// --- Statistical Benchmark System ---
#define BENCHMARK_WINDOW 50  // Rolling window of 50 samples

struct NoiseStats {
    float raw_history[BENCHMARK_WINDOW];
    float kf_history[BENCHMARK_WINDOW];
    uint16_t count;
    uint16_t index;
};

NoiseStats bench_stats[MAX_DRONES];

void initBenchmarkStats() {
    for (int i = 0; i < MAX_DRONES; i++) {
        memset(bench_stats[i].raw_history, 0, sizeof(bench_stats[i].raw_history));
        memset(bench_stats[i].kf_history, 0, sizeof(bench_stats[i].kf_history));
        bench_stats[i].count = 0;
        bench_stats[i].index = 0;
    }
}

void addBenchmarkSample(uint8_t peer_id, float raw_m, float kf_m) {
    if (peer_id >= MAX_DRONES) return;

    bench_stats[peer_id].raw_history[bench_stats[peer_id].index] = raw_m * 100.0f;
    bench_stats[peer_id].kf_history[bench_stats[peer_id].index]  = kf_m * 100.0f;

    bench_stats[peer_id].index = (bench_stats[peer_id].index + 1) % BENCHMARK_WINDOW;
    if (bench_stats[peer_id].count < BENCHMARK_WINDOW) {
        bench_stats[peer_id].count++;
    }
}

void computeBenchmark(uint8_t peer_id, float &raw_std_dev, float &kf_std_dev, float &noise_reduction_pct) {
    if (peer_id >= MAX_DRONES || bench_stats[peer_id].count < 10) {
        raw_std_dev = 0.0f;
        kf_std_dev = 0.0f;
        noise_reduction_pct = 0.0f;
        return;
    }

    uint16_t n = bench_stats[peer_id].count;
    float raw_sum = 0.0f, kf_sum = 0.0f;

    for (uint16_t i = 0; i < n; i++) {
        raw_sum += bench_stats[peer_id].raw_history[i];
        kf_sum  += bench_stats[peer_id].kf_history[i];
    }

    float raw_mean = raw_sum / n;
    float kf_mean  = kf_sum / n;

    float raw_sq_diff = 0.0f, kf_sq_diff = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        raw_sq_diff += powf(bench_stats[peer_id].raw_history[i] - raw_mean, 2);
        kf_sq_diff  += powf(bench_stats[peer_id].kf_history[i] - kf_mean, 2);
    }

    raw_std_dev = sqrtf(raw_sq_diff / (n - 1));
    kf_std_dev  = sqrtf(kf_sq_diff / (n - 1));

    if (raw_std_dev > 0.001f) {
        noise_reduction_pct = ((raw_std_dev - kf_std_dev) / raw_std_dev) * 100.0f;
    } else {
        noise_reduction_pct = 0.0f;
    }
}

// --- Frame Structure ---
typedef struct __attribute__((packed)) {
    uint8_t msg_type;
    uint8_t dest_id;
    uint8_t src_id;

    int64_t t_tx_poll;   
    int64_t t_rx_resp[MAX_DRONES];   
    int64_t t_tx_final;  
} UwbFrame;

// --- State Variables ---
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
float current_raw_distances_m[MAX_DRONES] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
float current_filtered_distances_m[MAX_DRONES] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

bool waiting_for_responses = false;
uint32_t poll_sent_micros = 0;
const uint32_t FINAL_TRIGGER_DELAY_US = 12000;

// --- Interrupt Handlers ---
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

// --- Radio Transmission Functions ---
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

    for (int i = 0; i < MAX_DRONES; i++) {
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
    
    if (f.dest_id != SELF_DRONE_ID && f.dest_id != BROADCAST_ID) { 
        receiver(); return; 
    }

    DW1000Time rxTime;
    DW1000.getReceiveTimestamp(rxTime);
    int64_t current_rx_ts = rxTime.getTimestamp();

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

            double tof_ticks = (t_round1 * t_round2 - t_reply1 * t_reply2) / (t_round1 + t_round2 + t_reply1 + t_reply2);
            float dist = (float)(tof_ticks * DISTANCE_PER_TICK);
            
            if (dist >= 0.0f && dist < 100.0f && f.src_id < MAX_DRONES) {
                current_raw_distances_m[f.src_id] = dist;
                current_filtered_distances_m[f.src_id] = updateKalmanTrack(kalman_filters[f.src_id], dist);
                addBenchmarkSample(f.src_id, dist, current_filtered_distances_m[f.src_id]);
            }
        }
        receiver();
    }
}

// --- Setup & Loop ---
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.printf("=== DRONE 0x%02X INITIALIZED ===\n", SELF_DRONE_ID);

    for (int i = 0; i < MAX_DRONES; i++) {
        initKalmanTrack(kalman_filters[i]);
    }
    initBenchmarkStats();

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

    // --- Output Benchmark & Status ---
    if (now_ms - last_display_time >= DISPLAY_INTERVAL_MS) {
        last_display_time = now_ms;
        bool has_connections = false;
        
        for (int i = 1; i < MAX_DRONES; i++) {
            if (i != SELF_DRONE_ID && current_filtered_distances_m[i] >= 0.0f) {
                float raw_sigma = 0.0f, kf_sigma = 0.0f, gain_pct = 0.0f;
                computeBenchmark(i, raw_sigma, kf_sigma, gain_pct);

                Serial.printf("[0x%02X] Raw: %6.1f cm (std: +/-%.2f) | KF: %6.1f cm (std: +/-%.2f) | Bruit: -%.1f%% | V: %+.2f m/s\n",
                              i,
                              current_raw_distances_m[i] * 100.0f,
                              raw_sigma,
                              current_filtered_distances_m[i] * 100.0f,
                              kf_sigma,
                              gain_pct,
                              kalman_filters[i].v);
                has_connections = true;
            }
        }
        
        if (!has_connections) {
            Serial.println("Searching for peers...");
        }
    }
}