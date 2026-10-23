#include "filtre.h"
#include <cstdint>
#include <math.h>

static float median(float d[7]) {
    static uint16_t size = 7;
    float temp[size];
    for (int i = 0; i < size; i++) {
        temp[i] = d[i];
    }

    for (int i = 1; i < size; i++) {
        float key = temp[i];
        int j = i - 1;
        while (j >= 0 && temp[j] > key) {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }
    if (size % 2 == 0) {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0f;
    } else {
        return temp[size / 2];
    }
}



void polynomial_filter(float* d) {
    float raw_cm = *d * 100.0f;
    
    // Equation d'erreur = a5*x^4 + a4x^3 + a3*x^2 + a2*x + a1
    const float a5 = -1.004e-6f; 
    const float a4 = 0.0005059f; 
    const float a3 = -0.09263f;  
    const float a2 = 7.398f;     
    const float a1 = -221.7;    

    float estimated_error_cm = a1 + raw_cm * (a2 + raw_cm * (a3 + raw_cm * (a4 + raw_cm * a5)));
    
    float corrected_cm = raw_cm - estimated_error_cm;

    if (corrected_cm < 0.0f) {
        corrected_cm = 0.0f;
    }

    *d = corrected_cm / 100.0f;
}

bool NLOS_filter(UWB_Diagnostics diag) {

    float min_fpp_threshold = -85.0f; // dBm (adjust based on your hardware calibration)
    if (diag.firstPathPower < min_fpp_threshold) {
        return false; 
    }

    float power_diff = fabsf(diag.receivePower - diag.firstPathPower);
    float max_power_diff = 6.0f; // dB max expected difference in clear LOS

    if (power_diff > max_power_diff) {
        return false; 
    }

    return true; 
}

bool innovationGate_filter(float current_d, float prev_d, float max_speed, float dt) {

    float max_possible_delta = max_speed * dt;

    float sensor_noise_margin = 0.2f; 
    float allowed_change = max_possible_delta + sensor_noise_margin;

    float actual_change = fabsf(current_d - prev_d);

    if (actual_change > allowed_change) {
        return false; 
    }

    return true; // Variation cohérente (acceptée)
}



