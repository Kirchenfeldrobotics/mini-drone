#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "freertos/semphr.h"
#include "esp_log.h"   
#include "driver/rmt_tx.h"
#include "pid.hpp"
#include "comms.hpp"
#include <iostream>
#include <algorithm> 

#define DSHOT_RES_HZ   10000000
#define T_BIT_TICKS    17
#define T1H_TICKS      13
#define T0H_TICKS      7
#define DSHOT_SYMBOLS 17

#define M1_PIN GPIO_NUM_25 // front right 
#define M2_PIN GPIO_NUM_26 // back left 
#define M3_PIN GPIO_NUM_27 // front left 
#define M4_PIN GPIO_NUM_28 // back right 

static const char* TAG = "DSHOT"; 

constexpr int IDLE_THROTTLE     = 100; 
constexpr int MAX_THROTTLE      = 2047; 
constexpr int MAX_THROTTLE_STEP = 2; 

class DShotMotor {
public: 
    DShotMotor() = default;
    DShotMotor(const DShotMotor&) = delete;
    DShotMotor& operator=(const DShotMotor&) = delete;

    ~DShotMotor() {
        if (channel_) {
            rmt_disable(channel_);
            rmt_del_channel(channel_);
        }
        if (encoder_) rmt_del_encoder(encoder_);
    }

    // Init motor for dshot comm
    esp_err_t init(gpio_num_t gpio) {
        esp_err_t ret; 
        
        // Configure
        rmt_tx_channel_config_t ch_cfg = {}; 
        ch_cfg.gpio_num          = gpio; 
        ch_cfg.clk_src           = RMT_CLK_SRC_DEFAULT; 
        ch_cfg.resolution_hz     = DSHOT_RES_HZ; 
        ch_cfg.mem_block_symbols = 64; 
        ch_cfg.trans_queue_depth = 4; 

        ret = rmt_new_tx_channel(&ch_cfg, &channel_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT TX channel: %d.", esp_err_to_name(ret)); 
            return ret; 
        } else ESP_LOGI(TAG, "RMT TX channel created."); 

        rmt_copy_encoder_config_t enc_cfg = {}; 
        ret = rmt_new_copy_encoder(&enc_cfg, &encoder_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT encoder: %d.", esp_err_to_name(ret)); 
            return ret; 
        } else ESP_LOGI(TAG, "RMT encoder created."); 

        ret = rmt_enable(channel_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT TX channel: %d.", esp_err_to_name(ret)); 
            return ret; 
        } else ESP_LOGI(TAG, "RMT TX channel enabled."); 

        return ESP_OK; 
    }

    // Send symbols via dshot to motor
    esp_err_t send(int throttle, bool telemetry = false) {
        // Build 16bit frame 
        const uint16_t frame = build_frame(throttle, telemetry); 
        // Encode frame to 17 symboles
        encode_frame(frame); 

        rmt_transmit_config_t tx_cfg = {}; 
        tx_cfg.loop_count = 0; 

        // transmit symboles to motors
        return rmt_transmit(channel_, encoder_, symbols_, sizeof(symbols_), &tx_cfg); 
    }

private: 
    // Build 16bit frame: 15-5 -> throttle value; 4 -> telemetry; 3-0 -> xor checksum 
    static uint16_t build_frame(uint16_t throttle,  bool telemetry) {
        uint16_t value = (throttle << 1) | (telemetry ? 1 : 0); 
        uint16_t crc  = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F; 
        return (value << 4) | crc; 
    }

    // Write symbols based on a frame: 
    void  encode_frame(uint16_t frame) {
        // Encode every bit of the frame
        for (int i = 0; i < 16; i++) {
            bool bit = (frame >> (15 - i)) & 0x01; 
            // Long HIGH -> 1; Short High -> 0. Every symbol is equally long in time
            symbols_[i].level0    = 1; 
            symbols_[i].duration0 = bit ? T1H_TICKS : T0H_TICKS; 
            symbols_[i].level1    = 0; 
            symbols_[i].duration1 = T_BIT_TICKS - (bit ? T1H_TICKS : T0H_TICKS); 
        }
        // LOW twice -> finished transmition
        symbols_[16].level0    = 0; 
        symbols_[16].duration0 = 20; 
        symbols_[16].level1    = 0; 
        symbols_[16].duration1 = 20;
    }

    rmt_channel_handle_t channel_ = nullptr; 
    rmt_encoder_handle_t encoder_ = nullptr; 
    rmt_symbol_word_t symbols_[DSHOT_SYMBOLS] = {}; 
}; 

// Send specfic thorttle to all motors
void send_all(DShotMotor* motors[4], int throttle) {
    for (int i = 0; i < 4; i++) motors[i]->send(throttle); 
}

void send_throttles(DShotMotor* motors[4], int throttles[4]) {
    for (int i = 0; i < 4; i++) {
        motors[i]->send(std::clamp(throttles[i], IDLE_THROTTLE, MAX_THROTTLE)); 
    }
}
        
void motor_task(void *pvParameters) {
    PID data = {}; 
    int throttles[4]; 
    
    DShotMotor m1, m2, m3, m4; 
    DShotMotor* motors[4] = {&m1, &m2, &m3, &m4}; 
    
    // Init motors
    if (m1.init(M1_PIN) != ESP_OK ||
        m2.init(M2_PIN) != ESP_OK ||
        m3.init(M3_PIN) != ESP_OK ||
        m4.init(M4_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motors."); 
        vTaskDelete(NULL); 
    }

    // Arming
    for (int i = 0; i < 200; i++ ){
        send_all(motors, 0); 
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }

    // Event --> Motor start
    
    int base_throttle   = 0; 
    int target_throttle = IDLE_THROTTLE ; 
    int diff            = 0; 

    while (true) {
        if (xQueueReceive(s_pid_data_queue, &data, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Read current target throttle
            if (xSemaphoreTake(s_target_base_throttle_mutex, pdMS_TO_TICKS(5))  == pdTRUE) {
                target_throttle = s_target_base_throttle; 
                xSemaphoreGive(s_target_base_throttle_mutex); 
            } else ESP_LOGW(TAG, "Failed to update base throttle (mutex not available)."); 
            
            
            target_throttle = std::clamp(target_throttle, IDLE_THROTTLE, MAX_THROTTLE); 
            
            // Ramp base thorttle
            diff = target_throttle - base_throttle; 
            if (diff > MAX_THROTTLE_STEP)       base_throttle += MAX_THROTTLE_STEP; 
            else if (diff < -MAX_THROTTLE_STEP) base_throttle -= MAX_THROTTLE_STEP; 
            else                                base_throttle = target_throttle; 

            // Mixer
            throttles[0] = base_throttle - data.pitch - data.roll + data.yaw; 
            throttles[1] = base_throttle + data.pitch + data.roll + data.yaw; 
            throttles[2] = base_throttle - data.pitch + data.roll - data.yaw; 
            throttles[3] = base_throttle + data.pitch - data.roll - data.yaw; 

            // Send throttle to motor
            send_throttles(motors, throttles); 
        } else {
            // Fall back if queue is empty
            send_throttles(motors, throttles); 
            ESP_LOGW(TAG, "Failed to update throttle (data from PID missing)."); 
        }
    }
}