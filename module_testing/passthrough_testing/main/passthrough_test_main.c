/*
 * Microphone test for INMP441 breakout board
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include <es8388.h>

// Local macros
#define I2S_SAMPLING_FREQ_HZ (40960)

// Allocate buffer
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define RX_BUFFER_LEN (2048) // Try bigger spec
int rxBuffer[RX_BUFFER_LEN * 2]; // x2 for L + R channels
int txBuffer[RX_BUFFER_LEN * 2];

static unsigned int rx_ovfl_hit = 0;
static unsigned int tx_ovfl_hit = 0;

const char* TAG = "main";

/**
 * @brief Callback for RX receive queue overflow. Increments running counter.
 * 
 * @param handle I2S channel handle (TX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool rx_rcvd_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    rx_ovfl_hit++;
    
    return false;
}

/**
 * @brief Callback for TX send queue overflow. Increments running counter.
 * 
 * @param handle I2S channel handle (TX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool tx_sent_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    tx_ovfl_hit++;
    
    return false;
}

void print_ovfl_counts(void* pvParameters)
{
    while (1) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        ESP_LOGI("Stats", "Overflow counters: TX = %0d, RX = %0d", tx_ovfl_hit, rx_ovfl_hit);
    }
}


void app_main(void)
{
    ESP_LOGI(TAG, "Initializing microphone I2S interface...");

    i2s_chan_handle_t mic_handle, aux_handle;

    es8388_config();
    es_i2s_init(&aux_handle, &mic_handle, I2S_SAMPLING_FREQ_HZ);

    i2s_event_callbacks_t cbs = {
        // .on_recv = rx_rcvd_callback,
        .on_recv = NULL,
        .on_recv_q_ovf = rx_rcvd_overflow,
        // .on_sent = tx_sent_callback,
        .on_sent = NULL,
        .on_send_q_ovf = tx_sent_overflow,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(mic_handle, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(aux_handle, &cbs, NULL));

    TaskHandle_t ovfl_task;

    ESP_LOGW(TAG, "Starting stats task...");
    BaseType_t xReturned = xTaskCreate(
        print_ovfl_counts, 
        "Overflow counter",
        4096u,
        NULL,
        0,
        &ovfl_task
    );

    // Enable channels
    ESP_LOGI(TAG, "Channels initiated! Beginning passthrough test...");
    ESP_ERROR_CHECK(i2s_channel_enable(mic_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(aux_handle));
    fflush(stdout);
    
    // Main loop
    while (1) {
        size_t bytes_read = 0, bytes_written = 0;
        esp_err_t ret_val;

        // Perform read
        ret_val = i2s_channel_read(mic_handle, &rxBuffer, sizeof(rxBuffer), &bytes_read, 5000);

        if (ret_val != ESP_OK || bytes_read != sizeof(rxBuffer)) {
            ESP_LOGW(TAG, "WARNING: read failed! Err code %d, %d bytes read", (int)ret_val, bytes_read);
            continue;
        }

        // Copy rxBuffer into txBuffer as is
        for (int i = 0; i < RX_BUFFER_LEN; i++) {
            txBuffer[2 * i] = rxBuffer[2 * i];
            txBuffer[2 * i + 1] = rxBuffer[2 * i]; // Same L+R
        }


        ret_val = i2s_channel_write(aux_handle, &txBuffer, bytes_read, &bytes_written, 5000);
        if (ret_val != ESP_OK || bytes_written != bytes_read) {
            ESP_LOGW(TAG, "WARNING: write failed! Err code %d, %d bytes written", (int)ret_val, bytes_written);
        }

        fflush(stdout);
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
