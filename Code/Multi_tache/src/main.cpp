#include <Arduino.h>

// Task running on Core 0
void taskUwbCore0(void *pvParameters) {
    uint32_t counter = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(500); // 2 Hz (every 500ms)

    for (;;) {
        counter++;
        Serial.printf("[CORE %d] UWB Task: acquisition #%u (Tick=%u)\n",
                      xPortGetCoreID(), counter, xTaskGetTickCount());

        vTaskDelayUntil(&lastWakeTime, period);
    }
}   

// Task running on Core 1
void taskControllerCore1(void *pvParameters) {
    uint32_t counter = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(200); // 5 Hz (every 200ms)

    for (;;) {
        counter++;
        Serial.printf("    -> [CORE %d] Controller Task: step #%u\n",
                      xPortGetCoreID(), counter);

        vTaskDelayUntil(&lastWakeTime, period);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1500); // Time to open the serial monitor

    Serial.println("\n--- Starting Dual-Core Multitasking Test ---");

    // Pin UWB simulation to Core 0
    xTaskCreatePinnedToCore(
        taskUwbCore0,       // Task function
        "UWB_Task",         // Task name
        2048,               // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority
        NULL,               // Task handle
        0                   // Core ID 0
    );

    // Pin Controller simulation to Core 1
    xTaskCreatePinnedToCore(
        taskControllerCore1, // Task function
        "Controller_Task",  // Task name
        2048,               // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority
        NULL,               // Task handle
        1                   // Core ID 1
    );
}

void loop() {
    // Arduino loop task is deleted so it doesn't consume CPU cycles
    vTaskDelete(NULL);
}