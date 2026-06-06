#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "freertos/semphr.h"
#include "esp_log.h"   
#include "mpu6050.hpp"
#include "./include/pid.hpp"
#include "comms.hpp"
#include <iostream>
#include <algorithm>
#include "flight_types.hpp"

#define KP 3.f
#define KD 15.f
#define KI 0.1f

#define KP_YAW 2.f 
#define KD_YAW 0.f 
#define KI_YAW 0.01f 

#define I_LIMIT 50.f

QueueHandle_t s_pid_data_queue = nullptr;

static const char* TAG = "PID";

struct ValuePair {
    float pitch, roll, yaw; 
}; 

void pid_task(void *pvParameters) {
    xEventGroupWaitBits(s_startup_event_group, MPU_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY); 
    
    DroneAngles data = {}; 
    PID outData      = {}; 
    
    ValuePair error      = {}; 
    ValuePair prev_error = {}; 
    bool first           = true; 
    
    ValuePair P = {};
    ValuePair D = {}; 
    ValuePair I = {}; 

    xEventGroupSetBits(s_startup_event_group, PID_READY_BIT); 

    while (true) {
        if (xQueueReceive(s_mpu_data_queue, &data, pdMS_TO_TICKS(5)) == pdTRUE) {
            if(xSemaphoreTake(s_target_angles_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                error.pitch = data.pitch - s_target_angles.pitch; 
                error.roll  = data.roll - s_target_angles.roll; 
                error.yaw   = data.yaw - s_target_angles.yaw; 
                xSemaphoreGive(s_target_angles_mutex);
            } else ESP_LOGW(TAG, "Failed to update PID error: Couldn't take mutex."); 
             
            if (data.delta_t <= 0) data.delta_t = 0.001f; 
            if (first) { prev_error = error; first = false; }

            P.pitch = error.pitch * KP; 
            P.roll  = error.roll * KP; 
            P.yaw   = error.yaw * KP_YAW;
            
            D.pitch = (error.pitch - prev_error.pitch) / data.delta_t * KD; 
            D.roll  = (error.roll - prev_error.roll) / data.delta_t * KD; 
            D.yaw   = (error.yaw - prev_error.yaw) / data.delta_t * KD_YAW; 

            I.pitch += error.pitch * data.delta_t * KI; 
            I.roll  += error.roll * data.delta_t * KI; 
            I.yaw   += error.yaw * data.delta_t * KI_YAW; 

            I.pitch = std::clamp(I.pitch, -I_LIMIT, I_LIMIT); 
            I.roll  = std::clamp(I.roll, -I_LIMIT, I_LIMIT); 
            I.yaw   = std::clamp(I.yaw , -I_LIMIT, I_LIMIT); 

            outData.pitch = P.pitch + D.pitch + I.pitch; 
            outData.roll  = P.roll + D.roll + I.roll;
            outData.yaw   = P.yaw + D.yaw + I.yaw;

            xQueueOverwrite(s_pid_data_queue, &outData); 

            prev_error = error; 
        } else {
            ESP_LOGW(TAG, "Missing data from MPU."); 
            xQueueOverwrite(s_pid_data_queue, &outData); 
        }
    }
}