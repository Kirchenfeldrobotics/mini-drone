#pragma once 

struct AnalyticsData {
    float vbat_volts; 
    float current_amps; 
    uint8_t battery_percentage; 
    bool low_percentage_warning; 
}; 

extern QueueHandle_t s_analytics_data_queue; 

void analytics_task(void *pvParameters);  

