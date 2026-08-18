#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t s_pid_data_queue;

struct TargetDroneAngles {
    float pitch, roll, yaw; 
}; 

struct AngleState {
    float pitch_angle; 
    float roll_angle; 
    float yaw_rate; 
}; 

struct PID {
    float pitch, roll, yaw; 
}; 

extern AngleState s_angle_state; 

void pid_task(void *pvParameters); 