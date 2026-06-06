#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "freertos/semphr.h"
#include "esp_log.h"   
#include "mpu6050.hpp"
#include "./include/pid.hpp"
#include "comms.hpp"

#define KP 0.1
#define KD 0.1 
#define KI 0.1 

#define I_LIMIT 50

struct ValuePair {
    float pitch, roll, yaw; 
}; 

void pid_task(void *pvParameters) {
    DroneAngles data = {}; 
    PID outData      = {}; 
    
    ValuePair error      = {}; 
    ValuePair prev_error = {}; 
    
    ValuePair P = {};
    ValuePair D = {}; 
    ValuePair I = {}; 

    s_pid_data_queue = xQueueCreate(1, sizeof(PID)); 

    while (true) {
        if (xQueueReceive(s_mpu_data_queue, &data, portMAX_DELAY)) {
            xSemaphoreTake(s_target_angels_mutex, portMAX_DELAY); 
            error.pitch = outData.pitch - s_target_angles.pitch; 
            error.roll  = outData.roll - s_target_angles.roll; 
            error.yaw   = outData.yaw - s_target_angles.yaw; 
            xSemaphoreGive(s_target_angels_mutex); 
            
            P.pitch = error.pitch * KP; 
            P.roll  = error.roll * KP; 
            P.yaw   = error.yaw * KP;
            
            D.pitch = (error.pitch - prev_error.pitch) / data.delta_t * KD; 
            D.roll  = (error.roll - prev_error.roll) / data.delta_t * KD; 
            D.yaw   = (error.yaw - prev_error.yaw) / data.delta_t * KD; 

            I.pitch += error.pitch * data.delta_t * KI; 
            I.roll  += error.roll * data.delta_t * KI; 
            I.yaw   += error.yaw * data.delta_t * KI; 

            if (I.pitch > I_LIMIT) I.pitch  = I_LIMIT;
            if (I.pitch > -I_LIMIT) I.pitch = -I_LIMIT; 
            if (I.roll > I_LIMIT) I.roll    = I_LIMIT; 
            if (I.roll > -I_LIMIT) I.roll   = -I_LIMIT; 
            if (I.yaw > I_LIMIT) I.yaw      = I_LIMIT; 
            if (I.yaw > -I_LIMIT) I.yaw     = -I_LIMIT; 

            outData.pitch = P.pitch + D.pitch + I.pitch; 
            outData.roll  = P.roll + D.roll + I.roll;
            outData.yaw   = P.yaw + D.yaw + I.yaw;

            xQueueSend(s_pid_data_queue, &outData, 0); 

            prev_error = error; 
        }
    }
}