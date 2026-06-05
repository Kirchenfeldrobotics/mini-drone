#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "freertos/semphr.h"
#include "./include/comms.hpp" 

// DO NOT START 

void udp_server_task(void *pvParameters) {
    s_target_angels_mutex = xSemaphoreCreateMutex(); 

    while (true) {
         xSemaphoreTake(s_target_angels_mutex, portMAX_DELAY); 
        // Write to 's_target_angles' to update target angles
        xSemaphoreGive(s_target_angels_mutex); 

    }   
}