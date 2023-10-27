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
#include "driver/dac_continuous.h"

#define DAC_SAMPLING_RATE (44000) // A little less than industry standard 44.1 kHz
#define WAVE_FREQ (440) // Concert A
#define WAVE_AMPLITUDE (255) // Will use L+R channel as differential input
#define SAMPLES_PER_CYCLE (DAC_SAMPLING_RATE / WAVE_FREQ)
uint8_t tx_buffer[SAMPLES_PER_CYCLE * 2]; 

void app_main(void)
{
    printf("Initializing DAC I2S interface\n");
    
    // Enable DAC
    dac_continuous_handle_t dac_handle;
    dac_continuous_config_t dac_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = 4,
        .buf_size  = SAMPLES_PER_CYCLE * 2,
        .freq_hz   = DAC_SAMPLING_RATE,
        .offset    = 0,
        .clk_src   = DAC_DIGI_CLK_SRC_APLL,
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_cfg, &dac_handle));
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));

    printf("Speaker channel initialized! Initializing wave...\n");

    // Fill TX buffer with sine wave data, since it should be the same per write
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        // Generate value based on sampling rate and A sine wave
        float t = 1.0 * i / DAC_SAMPLING_RATE; // Time is sample idx * period
        float sin_val_f = WAVE_AMPLITUDE * sin(2 * M_PI * WAVE_FREQ * t);
        int sin_val = (int)sin_val_f; // Cast to integer
        // Copy into upper-most byte.
        // Channel will be 0 or value depending on if negative/positive
        tx_buffer[2 * i]     = (sin_val_f > 0) ? (sin_val & 0xFF) : 0;
        tx_buffer[2 * i + 1] = tx_buffer[2 * i];
        // tx_buffer[2 * i + 1] = (sin_val_f < 0) ? ((-1 * sin_val) & 0xFF) << 24 : 0;
        printf("Wave idx %d: float %.5f => %d (TX 0x%8x)\n", i, sin_val_f, sin_val, tx_buffer[2 * i]);
    }
    printf("Filled TX buffer! Now sending data.");
    fflush(stdout);

    // Main loop
    while (1) {
        // Write to I2S
        ESP_ERROR_CHECK(dac_continuous_write(dac_handle, tx_buffer, sizeof(tx_buffer), NULL, 10000));
    }
    // Uninstall I2S
    dac_continuous_disable(dac_handle);

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
