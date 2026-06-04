#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "utils.hpp"


static EventGroupHandle_t s_wifi_event_group; 
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Wifi event gorup
static const char *TAG = "WIFI: "; 

// Reconnect config & state 
# define MAX_RETRY 5 
static int s_retry_num = 0; 



static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    esp_err_t ret; 
    
    // Esp is ready for connection ==> Connect to wifi
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Connecting to Wifi..."); 
        ret = esp_wifi_connect(); 
        if (ret != ESP_OK) ESP_LOGE(TAG, "Faild to connect to Wifi: %s.", esp_err_to_name(ret)); 
        else ESP_LOGI(TAG, "Wifi connected."); 
    // Esp lost connection ==> Attempt to reconnect
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wifi disconnected. Attempting to reconnect... (%d)", s_retry_num + 1); 
        if (s_retry_num < MAX_RETRY) {
            ret = esp_wifi_connect(); 
            if (ret != ESP_OK) ESP_LOGE(TAG, "Reconnect to wifi failed: %s.", esp_err_to_name(ret)); 
            else ESP_LOGI(TAG, "Wifi reconnected."); 
        } else {
            ESP_LOGE(TAG, "Too many reconnection attempts: Wifi connection failed."); 
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); 
        }   
    // Esp got an IP-addr ==> Set connected bit and print ip info 
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data; 
        ESP_LOGI(TAG, "Received IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0; 
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); 
    }
}

void wifi_init_sta(const char *ssid, const char *password) {
    // Blink LED while connecting
    TaskHandle_t blink_task_handle = NULL; 
    
    xTaskCreate(blink_task, "blink", 2048, nullptr, 5, &blink_task_handle); 
    
    esp_err_t ret; 
    
    // Create event group 
    s_wifi_event_group = xEventGroupCreate(); 


    // Create network interface: Connection between wifi driver and TCP/IP layer
    esp_netif_create_default_wifi_sta(); 

    // Init wifi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); 
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to initialize wifi driver: %s", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Wifi driver initialized."); 

    // Register event handlers
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL); 
    esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL); 

    // Configure Wifi
    wifi_config_t wifi_config = {}; 
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid)); 
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password)); 

    // Set wifi mode & config
    ret = esp_wifi_set_mode(WIFI_MODE_STA); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to set Wifi mode to STA: %s.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Wifi mode set to STA."); 
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to set Wifi config: %s.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Wifi config set."); 

    // Start wifi driver 
    ret = esp_wifi_start(); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to start Wifi driver: %s.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Successfully started wifi driver."); 

    // Wait for connection 
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY); 

    // Log & stop LED
    if (bits & WIFI_CONNECTED_BIT) ESP_LOGI(TAG, "Wifi connection has been successfully established.");
    else ESP_LOGE(TAG, "Wifi connection couldn't be established."); 

    vTaskDelete(blink_task_handle); 
}


