#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "esp_log.h"   
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "analytics.hpp"
#include "flight_types.hpp"

static const char* TAG = "Analytics"; 

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;

struct LipoPoint {
    float voltage; 
    uint8_t percent; 
}; 

static const LipoPoint LIPO_2S_HV_LUT[] = {
    {8.70f, 100},
    {8.50f,  95},
    {8.40f,  90},
    {8.30f,  85},
    {8.20f,  78},
    {8.10f,  70},
    {8.00f,  60},
    {7.90f,  50},
    {7.80f,  40},
    {7.70f,  30},
    {7.60f,  20},
    {7.50f,  12},
    {7.40f,   7},
    {7.20f,   3},
    {7.00f,   1},
    {6.60f,   0},
}; 
static const int LUT_SIZE = sizeof(LIPO_2S_HV_LUT) / sizeof(LIPO_2S_HV_LUT[0]);

#define VBAT_GPIO_PIN ADC_CHANNEL_7 
#define CURR_GPIO_PIN ADC_CHANNEL_6
#define ADC_ATTEN      ADC_ATTEN_DB_12
#define VBAT_DEVIDER  4.3f 
#define CURR_MV_PER_A 25.0f 

class EmaFilter {
public: 
    EmaFilter(float alpha) : alpha_(alpha), value_(0), initalized_(false) {}

    // Apply filter to new value
    float update(float new_sample) {
        if (!initalized_) {
            value_      = new_sample; 
            initalized_ = true; 
        } else value_ = alpha_ * new_sample + (1.0f - alpha_) * value_; 
        return value_; 
    }

    // get current value
    float get() const { return value_; }

private: 
    float alpha_; 
    float value_; 
    bool initalized_; 
}; 

void adc_init() {
    esp_err_t ret; 
    
    // Configure adc unit
    adc_oneshot_unit_init_cfg_t init_cfg; 
    init_cfg.unit_id = ADC_UNIT_1; 
    init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE; 
    ret = adc_oneshot_new_unit(&init_cfg, &adc_handle); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to initalize adc onshot unit: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Initalized adc onshot unit."); 

    // Configure channels
    adc_oneshot_chan_cfg_t channel_cfg; 
    channel_cfg.atten = ADC_ATTEN; 
    channel_cfg.bitwidth = ADC_BITWIDTH_DEFAULT; 
    ret = adc_oneshot_config_channel(adc_handle, VBAT_GPIO_PIN, &channel_cfg); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure VBAT adc channel: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Configured VBAT adc channel."); 
    ret = adc_oneshot_config_channel(adc_handle, CURR_GPIO_PIN, &channel_cfg); 
    if (ret != ESP_OK) ESP_LOGE(TAG, "Failed to configure CURR adc channel: %d.", esp_err_to_name(ret)); 
    else ESP_LOGI(TAG, "Configured CURR adc channel."); 
}

// Read raw adc values & caclulate volatage
int read_voltage_mv(adc_channel_t channel, int samples = 16) {
    int raw_sum = 0; 
    int raw_value; 

    // read raw adc values for multiple samples 
    for (int i = 0; i < samples; i++) {
        adc_oneshot_read(adc_handle, channel, &raw_value); 
        raw_sum += raw_value; 
    }
    
    // calculate voltage based on raw values
    int raw_average = raw_sum / samples; 
    int voltage_mv = (raw_average * 3100) / 4095; 

    return voltage_mv; 
}

// Convert voltage to percent
uint8_t voltage_to_percentage(float voltage) {
    if (voltage > LIPO_2S_HV_LUT[0].voltage) return 100; 
    if (voltage < LIPO_2S_HV_LUT[LUT_SIZE - 1].voltage) return 0; 

    for (int i = 0; i < LUT_SIZE - 1; i++) {
        float v_high = LIPO_2S_HV_LUT[i].voltage; 
        float v_low = LIPO_2S_HV_LUT[i].voltage; 
        if (voltage <= v_high && voltage > v_low) {
            uint8_t p_high = LIPO_2S_HV_LUT[i].percent; 
            uint8_t p_low = LIPO_2S_HV_LUT[i + 1].percent; 
            float ratio = (voltage - v_low) / (v_high - v_low); 
            return p_low + (uint8_t)(ratio * (p_high - p_low )); 
        }
    }
    return 0;
}

void analytics_task(void *pvParameters) {
    adc_init(); 
    
    EmaFilter vbat_filter(0.1f); 
    EmaFilter curr_filter(0.3f); 

    const TickType_t period = pdMS_TO_TICKS(10); 
    TickType_t last_wake    = xTaskGetTickCount(); 

    xEventGroupWaitBits(s_startup_event_group, UDP_SERVER_READY_BIT, pdFALSE, pdTRUE, portMAX_DELAY); 

    while (true) {
        // Read battery volts
        int vbat_mv         = read_voltage_mv(VBAT_GPIO_PIN); 
        float vbat_volts    = (vbat_mv * VBAT_DEVIDER) / 1000.0f; 
        float vbat_filtered = vbat_filter.update(vbat_volts); 

        // Read esc current
        int curr_mv         = read_voltage_mv(CURR_GPIO_PIN); 
        float curr_amps     = curr_mv / CURR_MV_PER_A; 
        float curr_filtered = curr_filter.update(curr_amps); 

        // Calculate percentage 
        uint8_t percentage = voltage_to_percentage(vbat_filtered); 
        AnalyticsData data = {}; 
        data.vbat_volts = vbat_filtered; 
        data.current_amps = curr_filtered; 
        data.battery_percentage = percentage; 
        data.low_percentage_warning = (vbat_filtered < 7.0f); 

        // Send data
        xQueueOverwrite(s_analytics_data_queue, &data); 

        vTaskDelayUntil(&last_wake, period); 
    }
}

