    #include "freertos/FreeRTOS.h"  
    #include "freertos/task.h"  
    #include "esp_log.h"   
    #include "esp_timer.h"
    #include "freertos/semphr.h"
    #include "./include/comms.hpp" 
    #include <atomic>
    #include <cstring>
    #include "flight_types.hpp"
    #include "analytics.hpp"
    #include "pid.hpp"

    #define DEBUG true

    SemaphoreHandle_t s_target_angles_mutex  = nullptr;
    TargetDroneAngles s_target_angles        = {};

    std::atomic<int> s_target_base_throttle{0};

    #define UDP_PORT        5555
    #define MAGIC_CONTROL   0x5E1FC9A3
    #define MAGIC_ANALYTICS 0x48EAD63
    #define PERIOD_MS       20
    #define RECV_TIMOUT     10

    static const char* TAG = "UDP";

    void udp_server_task(void *pvParameters) {
        xEventGroupWaitBits(s_startup_event_group, WIFI_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY); 
        
        // Create socket 
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); 
        if (sock < 0) { ESP_LOGE(TAG, "Failed to create socket: %d.", errno); vTaskDelete(NULL); }
        ESP_LOGI(TAG, "Socket created."); 

        // Set timout for socket 
        struct timeval timeout; 
        timeout.tv_sec = 0; 
        timeout.tv_usec = RECV_TIMOUT * 1000; 
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            ESP_LOGE(TAG, "Failed to set socket timeout."); 
            close(sock); 
            vTaskDelete(NULL); 
        }

        // Bind socket to port 
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

        ControlPacket rx_pkt; 
        struct sockaddr_in sender_addr; 
        socklen_t sender_addr_len = sizeof(sender_addr); 
        uint32_t last_sequence = 0; 

        struct sockaddr_in client_addr; 
        bool client_known = false; 
        uint32_t tx_sequence = 0; 
        int64_t last_analytics_us = 0; 

        xEventGroupSetBits(s_startup_event_group, UDP_SERVER_READY_BIT); 
        
        int n = 0; 
        while (true) {
            int len = recvfrom(sock, &rx_pkt, sizeof(rx_pkt), 0, (struct sockaddr*)&sender_addr, &sender_addr_len); 

            // Check for error in receive
            if (len < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                    ESP_LOGW(TAG, "Failed to receive data: %d.", errno);
                continue;
            }

            // Check packet size
            if (len != sizeof(ControlPacket)) {
                ESP_LOGW(TAG, "Received wrong packet size: %d (expected %d).", len, sizeof(ControlPacket)); 
                continue; 
            }

            // Check magic 
            if (rx_pkt.magic != MAGIC_CONTROL) {
                ESP_LOGW(TAG, "Received bad magic: 0x%08lX (expected: 0x5E1FC9A3).", rx_pkt.magic); 
                continue; 
            }

            // Store client addr for sending back
            client_addr  = sender_addr; 
            client_known = true; 

            // Check arrival consistency
            if (rx_pkt.sequence != last_sequence + 1 && last_sequence != 0) ESP_LOGW(TAG, "Packet gap: expected %lu, got %lu", last_sequence + 1, rx_pkt.sequence); 
            last_sequence = rx_pkt.sequence; 
            
            // Write data into storage
            xSemaphoreTake(s_target_angles_mutex, portMAX_DELAY); 
            s_target_angles.pitch = rx_pkt.target_pitch_angle; 
            s_target_angles.roll  = rx_pkt.target_roll_angle; 
            s_target_angles.yaw   = rx_pkt.target_yaw_rate; 
            xSemaphoreGive(s_target_angles_mutex); 

            s_target_base_throttle.store(rx_pkt.base_throttle);

    #if DEBUG 
            
        n++; 
        if (n >= 200) {
            ESP_LOGI(TAG, "--Received data--\n"
                        "  pitch:    %.2f\n"
                        "  roll:     %.2f\n"
                        "  yaw rate: %.2f\n"
                        "  throttle: %d\n"
                        "  armed:    %s",
                    rx_pkt.target_pitch_angle,
                    rx_pkt.target_roll_angle,
                    rx_pkt.target_yaw_rate,
                    rx_pkt.base_throttle,
                    (rx_pkt.flags & 0x01) ? "yes" : "no");
            n = 0; 
        }
        
    #endif 
            
            // Set eventgroup bits
            if (rx_pkt.flags & 0x01) xEventGroupSetBits(s_startup_event_group, MOTOR_CLEARANCE_BIT); 
            else xEventGroupClearBits(s_startup_event_group, MOTOR_CLEARANCE_BIT); 

            int64_t now_us = esp_timer_get_time(); 
            if (client_known && (now_us - last_analytics_us) >= PERIOD_MS * 1000) {

                last_analytics_us = now_us; 

                AnalyticsPacket tx_pkt = {}; 
                tx_pkt.magic = MAGIC_ANALYTICS; 
                tx_pkt.sequence = tx_sequence++; 
                if (xSemaphoreTake(s_analytics_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    tx_pkt.battery_voltage    = s_analytics_state.vbat_volts;
                    tx_pkt.battery_percentage = s_analytics_state.battery_percentage;  
                    tx_pkt.esc_amps           = s_analytics_state.current_amps; 
                    tx_pkt.pitch_angle        = s_angle_state.pitch_angle; 
                    tx_pkt.roll_angle         = s_angle_state.roll_angle; 
                    tx_pkt.yaw_rate           = s_angle_state.yaw_rate; 
                    xSemaphoreGive(s_analytics_mutex); 
                } else ESP_LOGW(TAG, "Couldn't take analytics mutex."); 

                int sent = sendto(sock, &tx_pkt, sizeof(tx_pkt), 0, (struct sockaddr*)&client_addr, sizeof(client_addr)); 
                if (sent < 0) ESP_LOGW(TAG, "Faild to send data: %d", errno); 
            
            }
        }   
    }