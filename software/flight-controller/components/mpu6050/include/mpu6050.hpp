#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

void mpu6050_task(void *pvParameters);
extern QueueHandle_t s_mpu_data_queue;

struct DroneAngles {
    float roll, pitch, yaw; 
    float delta_t; 
}; 
