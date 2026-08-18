#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

struct AnalyticsData {
    float vbat_volts; 
    float current_amps; 
    uint8_t battery_percentage; 
    bool low_percentage_warning; 
}; 

extern AnalyticsData     s_analytics_state;
extern SemaphoreHandle_t s_analytics_mutex;

void analytics_task(void *pvParameters);  

