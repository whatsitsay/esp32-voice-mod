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
#define I2S_SAMPLING_FREQ_HZ (44100)

// Allocate buffer
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define RX_BUFFER_LEN (4096) // Try bigger spec
int rxBuffer[RX_BUFFER_LEN * 2]; // x2 for L + R channels
int txBuffer[RX_BUFFER_LEN * 2];

const char* TAG = "main";


void app_main(void)
{
    ESP_LOGI(TAG, "Initializing microphone I2S interface...");

    i2s_chan_handle_t mic_handle, aux_handle;

    es8388_config();
    es_i2s_init(&aux_handle, &mic_handle, I2S_SAMPLING_FREQ_HZ);

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
        memcpy(txBuffer, rxBuffer, sizeof(txBuffer));


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
