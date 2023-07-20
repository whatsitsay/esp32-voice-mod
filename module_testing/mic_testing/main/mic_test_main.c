/*
 * Microphone test for INMP441 breakout board
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// Local macros
#define MIC_CLK_FREQ_HZ (44100)

// Allocate buffer
#define BYTES_PER_SAMPLE (2) // 16-bit (this doesn't seem right...should be 24)
#define RX_BUFFER_LEN (64)
int16_t rxBuffer[RX_BUFFER_LEN];


void app_main(void)
{
    printf("Initializing microphone I2S interface...\n");

    i2s_chan_handle_t mic_handle;
    // Init channel
    i2s_chan_config_t mic_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&mic_chan_cfg, NULL, &mic_handle);

    // Init config
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_CLK_FREQ_HZ),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_25,
            .ws   = GPIO_NUM_33,
            .dout = I2S_GPIO_UNUSED,
            .din  = GPIO_NUM_32,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(mic_handle, &std_cfg));

    // Enable mic RX channel
    ESP_ERROR_CHECK(i2s_channel_enable(mic_handle));
    printf("Microphone channel initialized! Receiving data...\n");
    fflush(stdout);
    
    // Main loop
    unsigned int iter_ctr = 0;
    while (1) {
        // Perform read
        size_t bytes_read = 0;
        esp_err_t ret_val = i2s_channel_read(
            mic_handle, &rxBuffer, RX_BUFFER_LEN * BYTES_PER_SAMPLE, &bytes_read, portMAX_DELAY
        );

        if (ret_val == ESP_OK) {
            // Read succeeded, calculate and print out average + num samples read
            int16_t samples_read = bytes_read / BYTES_PER_SAMPLE;
            float mean = 0;
            if (samples_read > 0) {
                for (int16_t i = 0; i < samples_read; i++) {
                    mean += rxBuffer[i];
                }
                mean /= samples_read;
            }
            // Print results to monitor
            printf("Iter %d INMP441 avg readout: %.2f (%d samples, first sample raw 0x%4x)\n", 
                    iter_ctr, mean, (int)samples_read, (unsigned int)rxBuffer[0]);
        }
        // Flush stdout
        fflush(stdout);

        // Delay slightly, half second
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // Increment running counter
        iter_ctr++;
    }

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
