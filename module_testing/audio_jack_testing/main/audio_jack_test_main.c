/*
 * Testing pushing audio data over I2S to PCM5102 DAC->Audio jack
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
#include "driver/i2s_std.h"

#define AUX_CLK_FREQ_HZ (44000)
#define WAVE_FREQ (440) // Concert A
#define SAMPLES_PER_CYCLE (AUX_CLK_FREQ_HZ / WAVE_FREQ)
#define WAVE_AMPLITUDE (16384.0) // 16-bit max / 2
int tx_buffer[SAMPLES_PER_CYCLE * 2]; // Double as each side is 32-bits

void app_main(void)
{
    printf("Initializing DAC I2S interface\n");
    
    i2s_chan_handle_t aux_handle;
    // Init channel
    i2s_chan_config_t aux_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&aux_chan_cfg, &aux_handle, NULL);

    // Initialize config
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUX_CLK_FREQ_HZ),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_17,
            .ws   = GPIO_NUM_4,
            .dout = GPIO_NUM_16,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(aux_handle, &std_cfg));

    // Enable aux TX channel
    ESP_ERROR_CHECK(i2s_channel_enable(aux_handle));

    printf("Speaker channel initialized! Initializing cycle of data...\n");

    // Fill TX buffer with sine wave data, since it should be the same per write
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
        // Generate value based on sampling rate and A sine wave
        float t = 1.0 * i / AUX_CLK_FREQ_HZ; // Time is sample idx * period
        float sin_val_f = WAVE_AMPLITUDE * sin(2 * M_PI * WAVE_FREQ * t);
        int16_t sin_val = (int16_t)sin_val_f;
        // Copy into buffer twice, for left and right
        // Upshift 16 to test 2B words
        tx_buffer[2 * i] = sin_val << 16;
        tx_buffer[2 * i + 1] = sin_val << 16;
        printf("Wave idx %d: float %.5f => %d (TX 0x%x)\n", i, sin_val_f, sin_val, tx_buffer[2*i]);
    }

    printf("Filled TX buffer! Now sending data.");
    fflush(stdout);

    // Main loop
    while (1) {
        // Write to I2S
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(aux_handle, tx_buffer, sizeof(tx_buffer), &bytes_written, portMAX_DELAY));

        if (bytes_written < sizeof(tx_buffer)) {
            printf("WARNING: Failed to write entire buffer! Only %d out of %d bytes\n",
                   (int)bytes_written, (int)sizeof(tx_buffer));
            fflush(stdout);
        }
    }

    // Uninstall I2S
    i2s_channel_disable(aux_handle);
    i2s_del_channel(aux_handle);

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
