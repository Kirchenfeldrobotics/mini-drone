#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "driver/gpio.h" 
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "wifi.hpp"
#include "./include/wifi_credentials.hpp"  
#include "utils.hpp"
#include "mpu6050.hpp"
#include "pid.hpp"
#include "comms.hpp"
#include "flight_types.hpp"
#include "dshot.hpp"

static const char *TAG = "MAIN";

EventGroupHandle_t s_startup_event_group;


// Init Non-Volatile storage
void init_nvs() {
    esp_err_t ret = nvs_flash_init(); 

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_LOGW(TAG, "Initalizing default nvs partition failed. Trying to earse and reinit..."); 
        ret = nvs_flash_erase(); 
        if (ret != ESP_OK) ESP_LOGE(TAG, "Erasing default nvs partition failed."); 
        ret = nvs_flash_init(); 
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "Error while initializing default nvs partition: %e.", ret); 
    else ESP_LOGI(TAG, "Initialized default nvs partition."); 
}



extern "C" void app_main(void) {
    esp_err_t ret; 

    // Create queues & mutexes
    s_mpu_data_queue = xQueueCreate(1, sizeof(DroneAngles)); 
    s_pid_data_queue = xQueueCreate(1, sizeof(PID)); 
    s_target_angles_mutex = xSemaphoreCreateMutex();

    // Create start-up event group: 
    s_startup_event_group = xEventGroupCreate(); 
    
    // Init nvs
    init_nvs(); 

    // Initializing TCP/IP Stack
    ret = esp_netif_init(); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to initialized TCP/IP Stack."); 
    else ESP_LOGI(TAG, "Initialized TCP/IP stack."); 

    // Create default event loop
    ret = esp_event_loop_create_default(); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to create default event loop: %e.", ret); 
    else ESP_LOGI(TAG, "Created default event loop."); 

    // Establish wifi connection 
    wifi_init_sta(WIFI_SSID, WIFI_PASSWORD);
    ESP_LOGI(TAG, "Wifi connection established."); 

    // Start UDP server
    xTaskCreatePinnedToCore(udp_server_task, "udp server task", 4096, nullptr, 5, nullptr, 0); 
    ESP_LOGI(TAG, "Launched UDP server."); 

    // Start MPU reader 
    xTaskCreatePinnedToCore(mpu6050_task, "mpu6050 task", 4096, nullptr, 10, nullptr, 1); 
    ESP_LOGI(TAG, "Started MPU reader."); 

    // Start PID calculation
    xTaskCreatePinnedToCore(pid_task, "pid task", 4096, nullptr, 10, nullptr, 1);
    ESP_LOGI(TAG, "Enabled PID calculation."); 

    // Start DShot communication
    xTaskCreatePinnedToCore(motor_task, "motor  task", 4096, nullptr, 10, nullptr, 1); 
    ESP_LOGI(TAG, "Established DShot communication."); 
}