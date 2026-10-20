#include <Arduino.h>
#include "multi_threading.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "uwb_driver.h"
#include "pixhawk_driver.h"
#include "esp_now_driver.h"
// Multi-threading suivant : 
//Coeur 1 : UWB et Communication 
//Coeur 0 :Correcteur et Pixhawk
static constexpr BaseType_t UWB_CORE     = 1;
static constexpr BaseType_t COM_CORE     = 1;
static constexpr BaseType_t PIXHAWK_CORE = 0;

// Liste de priorité des actions : 
static constexpr UBaseType_t PRIO_UWB      = 3;  
static constexpr UBaseType_t PRIO_PIXHAWK  = 2;
static constexpr UBaseType_t PRIO_COM     = 1;

// Tailles des piles : 
static constexpr uint32_t STACK_UWB     = 4096;
static constexpr uint32_t STACK_PIXHAWK = 4096;
static constexpr uint32_t STACK_COM     = 4096;

static constexpr uint32_t PIXHAWK_PERIOD_MS = 5;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 100;
static constexpr uint8_t  DIST_QUEUE_LEN    = 8;

struct DistanceMsg {
    uint8_t from;   // drone initiateur
    float   d;      // distance en mètres
};

 //File de communication: 
static QueueHandle_t s_dist_q = nullptr;   // distances : file de 8, FIFO
static QueueHandle_t s_att_q  = nullptr;   // attitude  : 1 case, écrasée à chaque fois
static QueueHandle_t s_gps_q  = nullptr;   // attitude  : 1 case, écrasée à chaque foi

//Tâches :

static void uwbTask(void*) {
    for (;;) {
        uwb_update();
        scheduler_update();
 
        uint8_t from;
        float d;
        while (uwb_get_distance(&from, &d)) {
            DistanceMsg m = { from, d };
            xQueueSend(s_dist_q, &m, 0);   
        }
 
        vTaskDelay(1);
    }
}

static void pixhawkTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
 
    for (;;) {
        pixhawk_update();
 
        Attitude a;
        if (pixhawk_get_attitude(&a)) {
            xQueueOverwrite(s_att_q, &a);  
        }
        GPS gps;
        if (pixhawk_get_gps(&gps)) {
            xQueueOverwrite(s_gps_q, &gps);   
        }
 
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PIXHAWK_PERIOD_MS));
    }
}

static void espnowTask(void*) {
    float last_d[UWB_MAX_DRONES];
    for (uint8_t i = 0; i < UWB_MAX_DRONES; i++) last_d[i] = -1.0f;   // -1 = pas de mesure
 
    TickType_t next_telem = xTaskGetTickCount();
 
    for (;;) {
        DistanceMsg m;
        if (xQueueReceive(s_dist_q, &m, pdMS_TO_TICKS(10)) == pdTRUE) {
            do {
                if (m.from >= 1 && m.from <= UWB_MAX_DRONES) last_d[m.from - 1] = m.d;
                send2drone(m.from, m.d);   
            } while (xQueueReceive(s_dist_q, &m, 0) == pdTRUE);
        }
 
        if ((int32_t)(xTaskGetTickCount() - next_telem) >= 0) {
            next_telem += pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
 
            Attitude a = {};   // reste à 0 tant que rien n'est reçu
            GPS      g = {};
            xQueuePeek(s_att_q, &a, 0);   // lit sans vider : la valeur reste disponible
            xQueuePeek(s_gps_q, &g, 0);
 
            send2base( m.d,a, g);
        }
    }
}


bool rtos_start() {
    s_dist_q = xQueueCreate(DIST_QUEUE_LEN, sizeof(DistanceMsg));
    s_att_q  = xQueueCreate(1, sizeof(Attitude));
    s_gps_q  = xQueueCreate(1, sizeof(GPS));
    if (!s_dist_q || !s_att_q || !s_gps_q) return false;
 
    BaseType_t ok_uwb = xTaskCreatePinnedToCore(
        uwbTask, "uwb", STACK_UWB, nullptr, PRIO_UWB, nullptr, UWB_CORE);
 
    BaseType_t ok_pix = xTaskCreatePinnedToCore(
        pixhawkTask, "pixhawk", STACK_PIXHAWK, nullptr, PRIO_PIXHAWK, nullptr, PIXHAWK_CORE);
 
    BaseType_t ok_net = xTaskCreatePinnedToCore(
        espnowTask, "espnow", STACK_COM, nullptr, PRIO_COM, nullptr, COM_CORE);
 
    return ok_uwb == pdPASS && ok_pix == pdPASS && ok_net == pdPASS;
}


bool rtos_get_distance(DistanceMsg* m, uint32_t timeout_ms) {
    return xQueueReceive(s_dist_q, m, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
 
bool rtos_get_attitude(Attitude* a, GPS* gps) {
    return xQueueReceive(s_att_q, a, 0) == pdTRUE;
}