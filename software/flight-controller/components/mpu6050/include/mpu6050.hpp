#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h" 

void mpu6050_task(void *pvParameters); 
static QueueHandle_t s_mpu_data_queue = nullptr; 

struct DroneAngles {
    float roll, pitch, yaw; 
    float delta_t; 
}; 
