#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t s_pid_data_queue;

struct TargetDroneAngles {
    float pitch, roll, yaw; 
}; 

struct PID {
    float pitch, roll, yaw; 
}; 

void pid_task(void *pvParameters); 