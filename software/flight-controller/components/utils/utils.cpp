#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "driver/gpio.h" 
#include "flight_types.hpp"

#define LED_PIN GPIO_NUM_2 

// LED blink task 
void blink_task(void *pvParameters) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    BlinkPattern type = *((BlinkPattern *)pvParameters); 
    int loop_count     = 0; 
    int loop_intervall = 0;
    int loop_delay     = 0;  

    if (type == LED_BLINKING) {
        loop_count     = 2; 
        loop_intervall = 500; 
    } else if (type == LED_WAITING) {
        loop_count     = 7; 
        loop_intervall = 200; 
        loop_delay     = 800; 

    }

    while (true) {
        for (int i = 1; i < loop_count; i++) {
            gpio_set_level(LED_PIN, i % 2); 
            vTaskDelay(pdMS_TO_TICKS(loop_intervall));
        }
        vTaskDelay(pdMS_TO_TICKS(loop_delay)); 
    }
}