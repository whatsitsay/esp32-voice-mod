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
#define I2S_SAMPLING_RATE (44000) // A little less than industry standard 44.1 kHz
#define WAVE_FREQ (440) // Concert A
#define WAVE_AMPLITUDE (256) // Will use L+R channel as differential input
#define SAMPLES_PER_CYCLE (I2S_SAMPLING_RATE / WAVE_FREQ)
int tx_buffer[SAMPLES_PER_CYCLE * 2]; // Double as each side is 32-bits

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
        .bits_per_sample  = I2S_BITS_PER_SAMPLE_32BIT, // DAC will only use upper 8 MSB
        .channel_format   = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count    = 2,
        .dma_buf_len      = SAMPLES_PER_CYCLE * 4,
        .use_apll         = false,
    };

    // Install driver
    ESP_ERROR_CHECK(i2s_driver_install(DAC_I2S_CH, &i2s_config, 0, NULL));
    // enable DAC output
    ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN));

    printf("Speaker channel initialized! Initializing wave...\n");

    // Fill TX buffer with sine wave data, since it should be the same per write
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        // Generate value based on sampling rate and A sine wave
        float t = 1.0 * i / I2S_SAMPLING_RATE; // Time is sample idx * period
        float sin_val_f = WAVE_AMPLITUDE * sin(2 * M_PI * WAVE_FREQ * t);
        int sin_val = (int)sin_val_f; // Cast to integer
        // Copy into upper-most byte.
        // Channel will be 0 or value depending on if negative/positive
        tx_buffer[2 * i]     = (sin_val_f > 0) ? (sin_val & 0xFF) << 24 : 0;
        tx_buffer[2 * i + 1] = (sin_val_f < 0) ? ((-1 * sin_val) & 0xFF) << 24 : 0;
        printf("Wave idx %d: float %.5f => %d (L 0x%8x, R 0x%8x)\n", i, sin_val_f, sin_val, tx_buffer[2*i], tx_buffer[2*i + 1]);
    }
    printf("Filled TX buffer! Now sending data.");
    fflush(stdout);

    // Main loop
    while (1) {
        // Write to I2S
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_write(DAC_I2S_CH, &tx_buffer, sizeof(tx_buffer), &bytes_written, 10000));

        if (bytes_written < sizeof(tx_buffer)) {
            printf("WARNING: Failed to write entire buffer! Only %d out of %d bytes\n",
                   (int)bytes_written, (int)sizeof(tx_buffer));
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
