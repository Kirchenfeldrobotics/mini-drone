#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "driver/i2c_master.h"

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

#define GYRO_CONV              500/32768.f 
#define ACCEL_CONV             4/32768.f 

#define LOG_DATA               true

struct MpuData {
    float accel_x, accel_y, accel_z; 
    float gyro_x, gyro_y, gyro_z; 
}; 


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
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure bus: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Bus configured."); 

    // register mpu to bus 
    i2c_device_config_t dev_config = {}; 
    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7; 
    dev_config.device_address  = MPU_ADDR; 
    dev_config.scl_speed_hz    = MPU_CLK_SPEED; 
    ret = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_mpu_handle); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to add mpu to bus: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Added mpu to bus."); 

    // Read who am i register from mpu
    uint8_t who_am_i = 0; 
    ret = mpu_read_byte(MPU_WHO_AM_I_REG, &who_am_i); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to read single byte from mpu register : %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Who am I = 0x%02X (expected: 0x68).", who_am_i); 

    // Wake up mpu 
    ret = mpu_write_byte(MPU_PWR_MGMT_1_REG, 0x00); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to write single byte to mpu register: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "MPU has been woken up."); 

    // Configure gyroscope 
    ret = mpu_write_byte(MPU_GYROSCOPE_REG, GYROSCOPE_FS << 3);
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure gyroscope: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Gyroscope has been configured."); 

    // Configure accelerometer 
    ret = mpu_write_byte(MPU_ACCELEROMETER_REG, ACCELEROMETER_FS << 3); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure accelerometer: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Acceleometer has been configured."); 
}

void mpu6050_task(void *pvParameters) {
    // Initialize mpu
    init_mpu(); 

    TickType_t last_wake_time = xTaskGetTickCount(); 
    const TickType_t period   = pdTICKS_TO_MS(100); 

    while (true) {
        vTaskDelayUntil(&last_wake_time, period); 
        
        MpuData data; 
        if (mpu_read(&data) == ESP_OK) {
            // Comm 
#if LOG_DATA 
            ESP_LOGI(TAG, "Acc: %.2f %.2f %.2f g   Gyro: %.1f %.1f %.1f deg/s", data.accel_x, data.accel_y, data.accel_z, data.gyro_x,  data.gyro_y,  data.gyro_z);
#endif
        } else {
            ESP_LOGW(TAG, "MPU read failed."); 
        }
    }
}

