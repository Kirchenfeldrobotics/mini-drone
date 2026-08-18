#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "./include/mpu6050.hpp"
#include "cmath"
#include "flight_types.hpp"

static const char* TAG = "MPU"; 

#define I2C_SDA_PIN            GPIO_NUM_21
#define I2C_SCL_PIN            GPIO_NUM_22
#define MPU_ADDR               0x68
#define MPU_CLK_SPEED          400000
#define MPU_WHO_AM_I_REG       0x75 
#define MPU_PWR_MGMT_1_REG     0x6B         // reg for wake up 
#define MPU_GYROSCOPE_REG      0x1B
#define MPU_ACCELEROMETER_REG  0x1C
#define MPU_RAW_DATA_START_REG 0x3B
#define GYROSCOPE_FS           0x01         // 01 --> +- 500 deg/s
#define ACCELEROMETER_FS       0x01         // 01 --> +- 4 g   

#define GYRO_CONV              500.f / 32768.f 
#define ACCEL_CONV             4.f / 32768.f 
#define RAD_TO_DEG             180.f / M_PI

#define LOG_DATA               true

struct MpuData {
    float accel_x, accel_y, accel_z; 
    float gyro_x, gyro_y, gyro_z; 
}; 


QueueHandle_t s_mpu_data_queue = nullptr;

static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_mpu_handle;

// Read single byte from mpu 
esp_err_t mpu_read_byte(uint8_t reg, uint8_t *out) {
    return i2c_master_transmit_receive(
        s_mpu_handle, 
        &reg, 1, 
        out, 1, 
        pdMS_TO_TICKS(100)
    ); 
}

// Write single byte to mpu 
esp_err_t mpu_write_byte(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value}; 
    
    return i2c_master_transmit(
        s_mpu_handle, 
        buf, 2,
        pdMS_TO_TICKS(100) 
    ); 
}

// Read raw data from mpu 
esp_err_t mpu_read(MpuData *out) {
    uint8_t reg = MPU_RAW_DATA_START_REG; 
    uint8_t buf[14]; 

    // Read register 14 register from start reg onwards: gyro, accel & temp; 2 bits each
    esp_err_t ret = i2c_master_transmit_receive(
        s_mpu_handle, 
        &reg, 1, 
        buf, 14, 
        pdMS_TO_TICKS(100)
    ); 
    if (ret != ESP_OK) return ret; 

    out->accel_x = ((int16_t)((buf[0] << 8) | buf[1])) * ACCEL_CONV;  
    out->accel_y = ((int16_t)((buf[2] << 8) | buf[3])) * ACCEL_CONV;
    out->accel_z = ((int16_t)((buf[4] << 8) | buf[5])) * ACCEL_CONV; 
    // 6&7 => temperature 
    out->gyro_x =  ((int16_t)((buf[8] << 8) | buf[9])) * GYRO_CONV; 
    out->gyro_y =  ((int16_t)((buf[10] << 8) | buf[11])) * GYRO_CONV; 
    out->gyro_z =  ((int16_t)((buf[12] << 8) | buf[13])) * GYRO_CONV; 

    return ESP_OK; 
}

// Init mpu 
void init_mpu() {
    esp_err_t ret;  

    // bus config 
    i2c_master_bus_config_t bus_config = {}; 
    bus_config.i2c_port                     = I2C_NUM_0; 
    bus_config.sda_io_num                   = I2C_SDA_PIN; 
    bus_config.scl_io_num                   = I2C_SCL_PIN; 
    bus_config.clk_source                   = I2C_CLK_SRC_DEFAULT; 
    bus_config.glitch_ignore_cnt            = 7; 
    bus_config.flags.enable_internal_pullup = true; 
    ret = i2c_new_master_bus(&bus_config, &s_bus_handle); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure bus: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Bus configured."); 

    // register mpu to bus 
    i2c_device_config_t dev_config = {}; 
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7; 
    dev_config.device_address  = MPU_ADDR; 
    dev_config.scl_speed_hz    = MPU_CLK_SPEED; 
    ret = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_mpu_handle); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to add mpu to bus: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Added mpu to bus."); 

    // Read who am i register from mpu
    uint8_t who_am_i = 0; 
    ret = mpu_read_byte(MPU_WHO_AM_I_REG, &who_am_i); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to read single byte from mpu register 0x75: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Who am I = 0x%02X (expected: 0x68).", who_am_i); 

    // Wake up mpu 
    ret = mpu_write_byte(MPU_PWR_MGMT_1_REG, 0x00); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to write single byte to mpu register 0x6B: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "MPU has been woken up."); 

    // configure filter
    ret = mpu_write_byte(0x1A, 0x03);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to write single byte to mpu register 0x1A: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "MPU has been woken up."); 

    // Configure gyroscope 
    ret = mpu_write_byte(MPU_GYROSCOPE_REG, GYROSCOPE_FS << 3);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure gyroscope 0x1B: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Gyroscope has been configured."); 

    // Configure accelerometer 
    ret = mpu_write_byte(MPU_ACCELEROMETER_REG, ACCELEROMETER_FS << 3); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure accelerometer 0x1C: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "Acceleometer has been configured."); 
}

void mpu6050_task(void *pvParameters) {
    xEventGroupWaitBits(s_startup_event_group, UDP_SERVER_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY); 
    
    init_mpu(); 

    DroneAngles outData = {}; 
    MpuData data; 

    uint64_t last_time = 0; 
    uint64_t now       = 0; 
    float delta_t      = 0; 

    TickType_t last_wake_time = xTaskGetTickCount(); 
    const TickType_t period   = pdMS_TO_TICKS(1); 

    xEventGroupSetBits(s_startup_event_group, MPU_READY_BIT); 

    while (true) {
        vTaskDelayUntil(&last_wake_time, period);  
        
        if (mpu_read(&data) == ESP_OK) {
            now = esp_timer_get_time(); 
            delta_t = (now - last_time) / 1.0e6f; 
            if (last_time == 0) { last_time = now; continue; }
            last_time = now;
            

            float accel_pitch = atan2f(data.accel_x, sqrtf(data.accel_y * data.accel_y + data.accel_z * data.accel_z)) * RAD_TO_DEG; 
            float accel_roll = atan2f(data.accel_y, sqrtf(data.accel_x * data.accel_x + data.accel_z * data.accel_z)) * RAD_TO_DEG;   
              
            outData.pitch = 0.98f * (outData.pitch + data.gyro_x * delta_t) + 0.02f * accel_pitch; 
            outData.roll = 0.98f * (outData.roll + data.gyro_y * delta_t) + 0.02f * accel_roll; 
            outData.yaw = data.gyro_z;
            outData.delta_t = delta_t; 
            
            xQueueOverwrite(s_mpu_data_queue, &outData);  

        } else {
            ESP_LOGW(TAG, "MPU read failed."); 
            continue; 
        }
    }
}

