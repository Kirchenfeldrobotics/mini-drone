#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "freertos/semphr.h"
#include "esp_log.h"   
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"     
#include "pid.hpp"
#include "comms.hpp"
#include <iostream>
#include <algorithm> 
#include "flight_types.hpp"
#include "utils.hpp"

#define DEBUG true

// DShot150 -> baud_rate = 150_000; resolution stays at 40 MHz
#define DSHOT_RES_HZ    40000000
#define DSHOT_BAUD_RATE 150000
#define DSHOT_POST_DELAY_US 50    

#define M1_PIN GPIO_NUM_25 // front right 
#define M2_PIN GPIO_NUM_26 // back left 
#define M3_PIN GPIO_NUM_27 // front left 
#define M4_PIN GPIO_NUM_14 // back right 

static const char* TAG = "DSHOT"; 

constexpr int DSHOT_CMD_MOTOR_STOP = 0;
constexpr int DSHOT_THROTTLE_MIN  = 48;
constexpr int IDLE_THROTTLE       = DSHOT_THROTTLE_MIN;
constexpr int MAX_THROTTLE        = 1247;
constexpr int MAX_THROTTLE_STEP   = 2;

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

        // Configure RMT TX channel
        rmt_tx_channel_config_t ch_cfg = {}; 
        ch_cfg.gpio_num          = gpio; 
        ch_cfg.clk_src           = RMT_CLK_SRC_DEFAULT; 
        ch_cfg.resolution_hz     = DSHOT_RES_HZ; 
        ch_cfg.mem_block_symbols = 64;
        ch_cfg.trans_queue_depth = 4; 

        ret = rmt_new_tx_channel(&ch_cfg, &channel_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
            return ret; 
        } else ESP_LOGI(TAG, "RMT TX channel created."); 

        // Install official DShot encoder
        dshot_esc_encoder_config_t enc_cfg = {}; 
        enc_cfg.resolution     = DSHOT_RES_HZ; 
        enc_cfg.baud_rate      = DSHOT_BAUD_RATE; 
        enc_cfg.post_delay_us  = DSHOT_POST_DELAY_US; 

        ret = rmt_new_dshot_esc_encoder(&enc_cfg, &encoder_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT encoder: %s", esp_err_to_name(ret));
            return ret; 
        } else ESP_LOGI(TAG, "RMT encoder created."); 

        ret = rmt_enable(channel_); 
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT TX channel: %s", esp_err_to_name(ret));
            return ret; 
        } else ESP_LOGI(TAG, "RMT TX channel enabled."); 

        return ESP_OK; 
    }

    // Send symbols via dshot to motor
    esp_err_t send(int throttle, bool telemetry = false) {
        // 1. Wait for the previous transmission to finish (timeout in ms)
        // This ensures we do not overwrite the payload while the RMT driver is still reading it.
        rmt_tx_wait_all_done(channel_, 10); 

        // 2. Safely update the persistent payload
        payload_.throttle      = static_cast<uint16_t>(throttle); 
        payload_.telemetry_req = telemetry; 

        rmt_transmit_config_t tx_cfg = {}; 
        tx_cfg.loop_count = 0; 

        // 3. Transmit (asynchronous)
        return rmt_transmit(channel_, encoder_, &payload_, sizeof(payload_), &tx_cfg); 
    }

private: 
    rmt_channel_handle_t channel_ = nullptr; 
    rmt_encoder_handle_t encoder_ = nullptr; 
    dshot_esc_throttle_t payload_ = {}; // FIXED: Payload is now persistent memory
};

// Send specfic thorttle to all motors
void send_all(DShotMotor* motors[4], int throttle) {
    for (int i = 0; i < 4; i++) {
        esp_err_t ret = motors[i]->send(throttle);
        if (ret != ESP_OK) {
            static int err_count = 0;
            if (++err_count % 100 == 0) {
                ESP_LOGW(TAG, "Motor %d send returned: %s (count: %d)", 
                         i, esp_err_to_name(ret), err_count);
            }
        }
    }
}

void send_throttles(DShotMotor* motors[4], int throttles[4]) {
    for (int i = 0; i < 4; i++) {
        motors[i]->send(std::clamp(throttles[i], IDLE_THROTTLE, MAX_THROTTLE)); 
    }
}
        
void motor_task(void *pvParameters) {
    xEventGroupWaitBits(s_startup_event_group, PID_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY); 

    PID data = {}; 
    int throttles[4] = {IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE};
    
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

    xEventGroupSetBits(s_startup_event_group, DSHOT_READY_BIT);

    // ESCs require continuous zero-throttle frames to arm. If MOTOR_CLEARANCE_BIT
    // is already set (web app armed before boot), the old loop skipped entirely.
    ESP_LOGI(TAG, "Arming ESCs (mandatory zero-throttle period)...");
    for (int i = 0; i < 1000; i++) {
        send_all(motors, 0);
        if (i % 500 == 0) ESP_LOGI(TAG, "Arming ESCs... %d/1000", i);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_LOGI(TAG, "ESC arming complete.");

    // Now wait for user arm command (keep ESCs alive with zero-throttle)
    int k = 0;
    while ((xEventGroupGetBits(s_startup_event_group) & MOTOR_CLEARANCE_BIT) == 0) {
        send_all(motors, 0);
        if (k >= 800) {
            ESP_LOGI(TAG, "Waiting for clearance...");
            k = 0;
        }
        k++;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    xEventGroupSetBits(s_startup_event_group, MOTOR_ARMED_BIT);

    int base_throttle   = DSHOT_THROTTLE_MIN;
    int target_throttle = 48;
    int diff            = 0;
    bool was_disarmed   = false;

    int n = 0;
    while (true) {
        EventBits_t bits = xEventGroupGetBits(s_startup_event_group);
        bool cleared = bits & MOTOR_CLEARANCE_BIT;

        if (!cleared) {
            base_throttle = DSHOT_THROTTLE_MIN;
            was_disarmed = true;
            send_all(motors, DSHOT_CMD_MOTOR_STOP);
            xQueueReceive(s_pid_data_queue, &data, pdMS_TO_TICKS(2));
            continue;
        }

        // Re-arm ESCs after disarm (they need zero-throttle frames again)
        if (was_disarmed) {
            ESP_LOGI(TAG, "Re-arming ESCs...");
            for (int i = 0; i < 500; i++) {
                send_all(motors, 0);
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            was_disarmed = false;
            ESP_LOGI(TAG, "Re-arming complete.");
        }

        if (xQueueReceive(s_pid_data_queue, &data, pdMS_TO_TICKS(5)) == pdTRUE) {
            target_throttle = std::clamp(s_target_base_throttle.load(), IDLE_THROTTLE, MAX_THROTTLE);

            // Ramp base throttle towards target
            diff = target_throttle - base_throttle;
            if (diff > MAX_THROTTLE_STEP)       base_throttle += MAX_THROTTLE_STEP;
            else if (diff < -MAX_THROTTLE_STEP) base_throttle -= MAX_THROTTLE_STEP;
            else                                base_throttle = target_throttle;

            // Mixer
            throttles[0] = base_throttle - data.pitch - data.roll + data.yaw;
            throttles[1] = base_throttle + data.pitch + data.roll + data.yaw;
            throttles[2] = base_throttle - data.pitch + data.roll - data.yaw;
            throttles[3] = base_throttle + data.pitch - data.roll - data.yaw;

            send_throttles(motors, throttles);

#if DEBUG 
            
        n++; 
        if (n >= 800) {
            ESP_LOGI(TAG, "--Sent Throttle--\n"
                        "  Motor 1: %d\n"
                        "  Motor 2: %d\n"
                        "  Motor 3: %d\n"
                        "  Motor 4: %d\n", 
                    throttles[0], 
                    throttles[1], 
                    throttles[2], 
                    throttles[3]); 
            n = 0; 
        }
        
#endif

        if (base_throttle >= IDLE_THROTTLE) xEventGroupSetBits(s_startup_event_group, MOTOR_READY_BIT);
        } else {
            send_throttles(motors, throttles);
            ESP_LOGW(TAG, "Failed to update throttle (data from PID missing).");
        }
    }
}