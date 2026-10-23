#pragma once
#include <stdint.h>

typedef struct {
    float firstPathPower; 
    float receivePower;   
    float firstPathAmp1; 
} UWB_Diagnostics;

void polynomial_filter(float* d);
bool NLOS_filter (UWB_Diagnostics diag);   
bool innovationGate_filter(float current_d, float prev_d, float max_speed, float dt);