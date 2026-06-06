#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "freertos/semphr.h"
#include "./include/comms.hpp" 
#include <atomic>
#include <cstring>

#define UDP_PORT 5555
#define MAGIC    0x5E1FC9A3

static const char* TAG = "UDP"; 

void udp_server_task(void *pvParameters) {
    // Create socket 
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
    if (sock < 0) { ESP_LOGE(TAG, "Failed to create socket: %d.", errno); vTaskDelete(NULL); }
    ESP_LOGI(TAG, "Socket created."); 

    // Pind socket to port 
    struct sockaddr_in local_addr = {}; 
    local_addr.sin_family      = AF_INET; 
    local_addr.sin_port        = htons(UDP_PORT); 
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY); 

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: %d.", errno); 
        close(sock); 
        vTaskDelete(NULL); 
    }
    ESP_LOGI(TAG, "Listening on UDP port %d.", UDP_PORT);

    ControlPacket pkt; 
    struct sockaddr_in sender_addr; 
    socklen_t sender_addr_len = sizeof(sender_addr); 
    uint32_t last_sequence = 0; 
    
    while (true) {
        int len = recvfrom(sock, &pkt, sizeof(pkt), 0, (struct sockaddr*)&sender_addr, &sender_addr_len); 

        if (len < 0) {
            ESP_LOGW(TAG, "Failed to receive data: %d.", errno); 
            vTaskDelay(pdMS_TO_TICKS(100)); 
            continue; 
        }

        if (len != sizeof(ControlPacket)) {
            ESP_LOGW(TAG, "Received wrong packet size: %d (expected %d).", len, sizeof(ControlPacket)); 
            continue; 
        }

        if (pkt.magic != MAGIC) {
            ESP_LOGW(TAG, "Received bad magic: 0x%08lX (expected: 0x5E1FC9A3).", pkt.magic); 
            continue; 
        }

        if (pkt.sequence != last_sequence + 1 && last_sequence != 0) ESP_LOGW(TAG, "Packet gap: expected %lu, got %lu", last_sequence + 1, pkt.sequence); 
        last_sequence = pkt.sequence; 
        
        xSemaphoreTake(s_target_angles_mutex, portMAX_DELAY); 
        s_target_angles.pitch = pkt.target_pitch_angle; 
        s_target_angles.roll  = pkt.target_roll_angle; 
        s_target_angles.yaw   = pkt.target_yaw_rate; 
        xSemaphoreGive(s_target_angles_mutex); 

        s_target_base_throttle.store(pkt.base_throttle); 

        // Event handler --> start motor 
    }   
}