/*
 * Microphone test for INMP441 breakout board
 */

#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_dsp.h"

// Local macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
// Number of sample buffers per FFT calc
#define FFT_SAMPLING_COUNT (5)

// Allocate buffer
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define RX_BUFFER_LEN (4096) // Needs to be power of 2 for DSP lib
int rxBuffer[RX_BUFFER_LEN * 2]; // x2 for L + R channels

// FFT buffers
__attribute__((aligned(16))) float hann_win[RX_BUFFER_LEN];
__attribute__((aligned(16))) float rx_FFT[RX_BUFFER_LEN * 2]; // Will be complex
float rx_FFT_mag_raw[RX_BUFFER_LEN/2]; // dB magnitude. Half for relevant portion

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing microphone I2S interface...");

    i2s_chan_handle_t mic_handle;
    // Init channel
    i2s_chan_config_t mic_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
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
    ESP_ERROR_CHECK(i2s_channel_enable(mic_handle));

    ESP_LOGI(TAG, "Finished initiating mic I2S, setting up FFT coefficients");
    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, RX_BUFFER_LEN));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, RX_BUFFER_LEN);
    
    // Main loop
    while (1) {
        size_t bytes_read = 0;
        esp_err_t ret_val;

        for (int i = 0; i < FFT_SAMPLING_COUNT; i++) {
            // Perform read
            ret_val = i2s_channel_read(mic_handle, &rxBuffer, sizeof(rxBuffer), &bytes_read, 5000);

            if (ret_val != ESP_OK || bytes_read != sizeof(rxBuffer)) {
                ESP_LOGW(TAG, "Read failed! Err code %d, %d bytes read", (int)ret_val, bytes_read);
                continue;
            }
        }

        // Fill FFT buff with last sample
        for (int i = 0; i < RX_BUFFER_LEN; i++) {
            // Cast to signed int
            // Skip every other, as it should only be L channel with data
            int rx_val = rxBuffer[2 * i];
            // Cut out lower 8 bits and convert to float
            // Avoids overflow
            float rx_f = (float)(rx_val / 256);
            
            // Dot-multiply with Hann windows to reduce effect of edges
            rx_FFT[2 * i] = rx_f * hann_win[i];
            rx_FFT[2 * i + 1] = 0; // No complex component
        }

        // Calculate FFT, profile cycle count
        unsigned int start_b = dsp_get_cpu_cycle_count();
        dsps_fft2r_fc32_ae32(rx_FFT, RX_BUFFER_LEN);
        unsigned int end_b = dsp_get_cpu_cycle_count();
        float fft_comp_time_ms = (float)(end_b - start_b)/240e3;

        // Bit reverse
        dsps_bit_rev_fc32(rx_FFT, RX_BUFFER_LEN);
        // Convert to two complex vectors
        // Accounts for complex conjugate symmetry
        dsps_cplx2reC_fc32(rx_FFT, RX_BUFFER_LEN);

        // Calculate amplitude in dB
        for (int i = 0; i < RX_BUFFER_LEN / 2; i++) {
            float re_val = rx_FFT[i * 2];
            float im_val = rx_FFT[i * 2 + 1];

            rx_FFT_mag_raw[i] = sqrtf((re_val * re_val) + (im_val * im_val))/(float)RX_BUFFER_LEN;
        }
        
        // Show results
        ESP_LOGI(TAG, "Mic spectra magnitude (raw) (up to < 3 kHz)");
        dsps_view(rx_FFT_mag_raw, 256, 128, 10, 0, 2500000, '|');
        ESP_LOGI(TAG, "FFT for %i complex points take %i cycles (%.3f ms)", RX_BUFFER_LEN, end_b - start_b, fft_comp_time_ms);
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
