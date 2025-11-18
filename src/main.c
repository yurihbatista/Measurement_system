#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "driver/uart.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "INTERRUPT_SCOPE";

#define ADC_INPUT_CHANNEL   ADC_CHANNEL_6 // GPIO34
#define ADC_SAMPLE_RATE     40000
#define SERIAL_BAUD_RATE    4000000
#define ADC_READ_LEN        (256 * 8)

const uint16_t SYNC_WORD = 0xAAAA;
const uint16_t SAMPLES_PER_BLOCK = ADC_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool adc_calibrated = false;

static QueueHandle_t s_adc_queue;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t must_yield = pdFALSE;

    xQueueSendFromISR(s_adc_queue, edata, &must_yield);

    return (must_yield == pdTRUE);
}

static bool check_and_init_adc_calibration(adc_unit_t adc_unit, adc_cali_handle_t *out_handle) {
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = adc_unit,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_config, out_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Scheme: Line Fitting");
        return true;
    }
    ESP_LOGW(TAG, "ADC Calibration failed to initialize.");
    return false;
}

static void oscilloscope_task(void *arg) {
    uint16_t *tx_buffer = (uint16_t *)malloc(SAMPLES_PER_BLOCK * sizeof(uint16_t));
    if (!tx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for tx_buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Oscilloscope task started. Waiting for data from ADC interrupt...");

    adc_continuous_evt_data_t evt_data;

    while(1) {
        if (xQueueReceive(s_adc_queue, &evt_data, portMAX_DELAY)) {

            uint8_t *raw_adc_buffer = evt_data.conv_frame_buffer;
            uint32_t samples_read = evt_data.size / SOC_ADC_DIGI_RESULT_BYTES;

            for (int i = 0; i < samples_read; i++) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&raw_adc_buffer[i * SOC_ADC_DIGI_RESULT_BYTES];
                int voltage_mv = 0;
                if (adc_calibrated) {
                    adc_cali_raw_to_voltage(adc_cali_handle, p->type1.data, &voltage_mv);
                } else {
                    voltage_mv = p->type1.data; 
                }
                tx_buffer[i] = (uint16_t)voltage_mv;
            }

            uart_write_bytes(UART_NUM_0, (const char*)tx_buffer, samples_read * sizeof(uint16_t));
        }
    }
    free(tx_buffer);
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    adc_calibrated = check_and_init_adc_calibration(ADC_UNIT_1, &adc_cali_handle);

    uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUD_RATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    s_adc_queue = xQueueCreate(2, sizeof(adc_continuous_evt_data_t));

    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 4096, .conv_frame_size = ADC_READ_LEN, };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));
    
    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12, .channel = ADC_INPUT_CHANNEL, .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_RATE, .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, .pattern_num = 1, .adc_pattern = &adc_pattern,
    };
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    xTaskCreatePinnedToCore(oscilloscope_task, "scope_task", 4096, NULL, 5, NULL, 1);
    
    ESP_LOGI(TAG, "Initialization complete. Streaming INTERRUPT-DRIVEN data.");
}