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
#include "driver/dac_continuous.h"

// Local macros
#define I2S_SAMPLING_FREQ_HZ (44100)

// Allocate buffer
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define RX_BUFFER_LEN (1024) // Try bigger spec
int rxBuffer[RX_BUFFER_LEN * 2]; // x2 for L + R channels
uint8_t txBuffer[RX_BUFFER_LEN * 2];


void app_main(void)
{
    printf("Initializing microphone I2S interface...\n");

    i2s_chan_handle_t mic_handle;
    // Init channel
    i2s_chan_config_t mic_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_new_channel(&mic_chan_cfg, NULL, &mic_handle);

    // Initialize config
    i2s_std_config_t mic_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLING_FREQ_HZ),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_27,
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
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(mic_handle, &mic_cfg));

    printf("Initializing internal DAC I2S interface\n");
    
    // Init DAC channel
    dac_continuous_handle_t speaker_handle;
    dac_continuous_config_t speaker_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,
        .desc_num  = 4,
        .buf_size  = RX_BUFFER_LEN * 2,
        .freq_hz   = I2S_SAMPLING_FREQ_HZ,
        .offset    = 0,
        .clk_src   = DAC_DIGI_CLK_SRC_APLL,
        .chan_mode = DAC_CHANNEL_MODE_ALTER,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&speaker_cfg, &speaker_handle));

    // Enable channels
    printf("Channels initiated! Beginning passthrough test...\n");
    ESP_ERROR_CHECK(i2s_channel_enable(mic_handle));
    ESP_ERROR_CHECK(dac_continuous_enable(speaker_handle));
    fflush(stdout);
    
    // Main loop
    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret_val;

        // Perform read
        ret_val = i2s_channel_read(mic_handle, &rxBuffer, sizeof(rxBuffer), &bytes_read, 5000);

        if (ret_val != ESP_OK || bytes_read != sizeof(rxBuffer)) {
            printf("WARNING: read failed! Err code %d, %d bytes read\n", (int)ret_val, bytes_read);
            continue;
        }

        // Copy rxBuffer upper byte into txBuffer
        for (int i = 0; i < bytes_read / 4; i++) {
            // INMP441 returns stream of 32-bit values, we only need 8 MSB
            // If value is positive, grab 9 bits and exclude MSB (sign)
            // If value is negative, set to 0
            txBuffer[i] = (rxBuffer[i] > 0) ? (uint8_t)(rxBuffer[i] >> 24) : 0;
        }

        // Perform write of same buffer to DAC
        ret_val = dac_continuous_write(speaker_handle, txBuffer, sizeof(txBuffer), NULL, 10000);
        if (ret_val != ESP_OK ) {
            printf("WARNING: write failed! Err code %d\n", (int)ret_val);
        }

        fflush(stdout);
    }

    // If this point is hit, wait some time then restart
    printf("Exited main loop! Restarting ESP in 5 seconds...\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
