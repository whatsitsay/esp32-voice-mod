/*
 * Speaker test using internal DAC and 4 ohm/3 watt speaker
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "driver/dac.h"

#define DAC_I2S_CH (I2S_NUM_0)
#define DAC_MAX (255) // 8-bit DAC
#define I2S_SAMPLING_RATE (44100) // A little less than industry standard 44.1 kHz
#define WAVE_FREQ (440) // Concert A
#define TX_BUFFER_LEN (32)

void app_main(void)
{
    printf("Initializing DAC I2S interface\n");
    
    // Enable DAC
    dac_output_enable(DAC_CHANNEL_1);
    dac_i2s_enable(); // Enable DAC I2S

    // Setup I2S config
    // Need to use legacy config, as there is no indication on how to set
    // up standard config using built-in DAC
    i2s_config_t i2s_config = {
        .mode             = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate      = I2S_SAMPLING_RATE, // Industry standard
        .bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT, // DAC will only use upper 8 MSB
        .channel_format   = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count    = 2,
        .dma_buf_len      = TX_BUFFER_LEN,
        .use_apll         = false,
    };

    // Install driver
    ESP_ERROR_CHECK(i2s_driver_install(DAC_I2S_CH, &i2s_config, 0, NULL));
    // enable DAC output
    ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN));

    printf("Speaker channel initialized! Sending data...\n");
    fflush(stdout);

    // Main loop
    int wave_idx = 0;
    while (1) {
        // Fill TX buffer with sine wave data
        // Generate value based on sampling rate and A sine wave
        // Value should always be positive
        // Calculate sine value
        float t = 1.0 * wave_idx / I2S_SAMPLING_RATE;
        int sin_val = (int)((DAC_MAX / 2) * sin(2 * M_PI * WAVE_FREQ * t));
        // Increase by half
        sin_val += DAC_MAX / 2;
        // Mask for internal DAC
        sin_val = (sin_val & 0xff) << 8;
        // Copy twice into buffer, 2B apart, for L + R channels
        uint32_t tx_data = (sin_val << 16) | sin_val;

        // Write to I2S
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_write(DAC_I2S_CH, &tx_data, sizeof(tx_data), &bytes_written, 1000));

        if (bytes_written < sizeof(tx_data)) {
            printf("WARNING: Failed to write entire buffer! Only %d out of %d bytes\n",
                   (int)bytes_written, (int)sizeof(tx_data));
            fflush(stdout);
        }
    }
    // Uninstall I2S
    i2s_driver_uninstall(DAC_I2S_CH);

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
