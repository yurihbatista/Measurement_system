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
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "INTERRUPT_SCOPE";

#define DOWNSAMPLING_FACTOR 8
#define ADC_INPUT_CHANNEL_0   ADC_CHANNEL_6 // GPIO34
#define ADC_INPUT_CHANNEL_1   ADC_CHANNEL_7 //GPIO35
#define ADC2_INPUT_CHANNEL_0  ADC_CHANNEL_8 // GPIO25
#define ADC2_INPUT_CHANNEL_1  ADC_CHANNEL_9 // GPIO26

#define ADC_SAMPLE_RATE     100000
#define ADC2_SAMPLE_RATE_HZ 100
#define SERIAL_BAUD_RATE    4000000
#define ADC_READ_LEN        (256 * 8)

const uint16_t SYNC_WORD = 0xAAAA;
const uint16_t SAMPLES_PER_BLOCK = ADC_READ_LEN / SOC_ADC_DIGI_RESULT_BYTES;

static adc_continuous_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc2_handle;
static bool adc_calibrated = false;

static QueueHandle_t s_adc_queue;
static QueueHandle_t uart_queue;
static TaskHandle_t s_adc2_task_handle;


typedef struct {
    uint16_t start_word;
    uint8_t channel_id;
    uint16_t data_size;
} __attribute__((packed)) PacketHeader;

const uint16_t END_WORD = 0xBBBB;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t must_yield = pdFALSE;
    xQueueSendFromISR(s_adc_queue, edata, &must_yield);
    return (must_yield == pdTRUE);
}

static void adc2_timer_callback(void* arg) {
    if (s_adc2_task_handle != NULL) {
        xTaskNotifyGive(s_adc2_task_handle);
    }
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
    uint16_t *tx_buffer_0 = (uint16_t *)malloc((SAMPLES_PER_BLOCK/2) * sizeof(uint16_t));
    uint16_t *tx_buffer_1 = (uint16_t *)malloc((SAMPLES_PER_BLOCK/2) * sizeof(uint16_t));
    if (!tx_buffer_0 || !tx_buffer_1) {
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

            uint32_t downsampled_samples_count[2] = {0};
            for (int i = 0; i < samples_read; i += DOWNSAMPLING_FACTOR) {
                uint32_t sum[2] = {0};
                uint32_t count[2] = {0};
                for (int j = 0; j < DOWNSAMPLING_FACTOR && (i + j) < samples_read; j++) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&raw_adc_buffer[(i + j) * SOC_ADC_DIGI_RESULT_BYTES];
                    int voltage_mv = 0;
                    if(p->type1.channel == ADC_INPUT_CHANNEL_0) {
                          adc_cali_raw_to_voltage(adc_cali_handle, p->type1.data, &voltage_mv);
                          sum[0] += voltage_mv;
                          count[0]++;
                    }
                    if(p->type1.channel == ADC_INPUT_CHANNEL_1) {
                          adc_cali_raw_to_voltage(adc_cali_handle, p->type1.data, &voltage_mv);
                          sum[1] += voltage_mv;
                          count[1]++;
                    }
                }
                if(count[0]) tx_buffer_0[downsampled_samples_count[0]++] = (uint16_t)(sum[0] / count[0]);
                if(count[1]) tx_buffer_1[downsampled_samples_count[1]++] = (uint16_t)(sum[1] / count[1]);
            }

            PacketHeader header0 = {
                .start_word = SYNC_WORD,
                .channel_id = 0,
                .data_size = downsampled_samples_count[0] * sizeof(uint16_t)
            };
            uart_write_bytes(UART_NUM_0, (const char*)&header0, sizeof(PacketHeader));
            uart_write_bytes(UART_NUM_0, (const char*)tx_buffer_0, header0.data_size);
            uart_write_bytes(UART_NUM_0, (const char*)&END_WORD, sizeof(END_WORD));
            
            PacketHeader header1 = {
                .start_word = SYNC_WORD,
                .channel_id = 1,
                .data_size = downsampled_samples_count[1] * sizeof(uint16_t)
            };
            uart_write_bytes(UART_NUM_0, (const char*)&header1, sizeof(PacketHeader));
            uart_write_bytes(UART_NUM_0, (const char*)tx_buffer_1, header1.data_size);
            uart_write_bytes(UART_NUM_0, (const char*)&END_WORD, sizeof(END_WORD));
        }
    }
    free(tx_buffer_0);
    free(tx_buffer_1);
    vTaskDelete(NULL);
}

static void adc2_task(void *arg)
{
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int adc_raw[2];
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_INPUT_CHANNEL_0, &adc_raw[0]));
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC2_INPUT_CHANNEL_1, &adc_raw[1]));
        
        uint16_t adc_val_0 = (uint16_t)adc_raw[0];
        uint16_t adc_val_1 = (uint16_t)adc_raw[1];

        PacketHeader header2 = {
            .start_word = SYNC_WORD,
            .channel_id = 2,
            .data_size = sizeof(uint16_t)
        };
        uart_write_bytes(UART_NUM_0, (const char*)&header2, sizeof(PacketHeader));
        uart_write_bytes(UART_NUM_0, (const char*)&adc_val_0, header2.data_size);
        uart_write_bytes(UART_NUM_0, (const char*)&END_WORD, sizeof(END_WORD));

        PacketHeader header3 = {
            .start_word = SYNC_WORD,
            .channel_id = 3,
            .data_size = sizeof(uint16_t)
        };
        uart_write_bytes(UART_NUM_0, (const char*)&header3, sizeof(PacketHeader));
        uart_write_bytes(UART_NUM_0, (const char*)&adc_val_1, header3.data_size);
        uart_write_bytes(UART_NUM_0, (const char*)&END_WORD, sizeof(END_WORD));
    }
}


void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    adc_calibrated = check_and_init_adc_calibration(ADC_UNIT_1, &adc_cali_handle);

    uart_config_t uart_config = {
        .baud_rate = SERIAL_BAUD_RATE, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 4096, 5, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    s_adc_queue = xQueueCreate(4, sizeof(adc_continuous_evt_data_t));

    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = ((ADC_READ_LEN/2)*16), .conv_frame_size = ADC_READ_LEN, };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));
    
    adc_digi_pattern_config_t adc_pattern[2] = {0};

    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = ADC_INPUT_CHANNEL_0;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;

    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = ADC_INPUT_CHANNEL_1;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_SAMPLE_RATE, 
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1, 
        .pattern_num = 2, 
        .adc_pattern = adc_pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL));

    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    // ADC2 setup
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_INPUT_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC2_INPUT_CHANNEL_1, &config));


    xTaskCreatePinnedToCore(oscilloscope_task, "scope_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(adc2_task, "adc2_task", 2048, NULL, 5, &s_adc2_task_handle, 1);

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &adc2_timer_callback,
            .name = "adc2_timer"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000 / ADC2_SAMPLE_RATE_HZ));

    
    ESP_LOGI(TAG, "Initialization complete. Streaming INTERRUPT-DRIVEN data.");
}