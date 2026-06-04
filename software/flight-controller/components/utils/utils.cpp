#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "driver/gpio.h" 

// LED blink task 
void blink_task(void *pvParameters) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    while (true) {
        for (int i = 1; i > -1; i--) {
            gpio_set_level(LED_PIN, i); 
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}