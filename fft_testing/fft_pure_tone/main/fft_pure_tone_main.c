/*
 * Test of FFT deconstruction/iFFT reconstruction for pure sine wave
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_task_wdt.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"

// Local macros
#define TONE_FREQ_HZ (440.0) // Concert A
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define TONE_FREQ_ARR (TONE_FREQ_HZ / I2S_SAMPLING_FREQ_HZ)

// Allocate buffer
#define SAMPLES_PER_AVG (50)
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define N_SAMPLES (2048) // Smaller for debugging
#define RX_BUFFER_LEN (N_SAMPLES / 2) // Try bigger spec
int N = N_SAMPLES;

// FFT buffers
__attribute__((aligned(16))) float hann_win[N_SAMPLES];
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Instantiate pointers to debug buffers
float tone_buffer[RX_BUFFER_LEN];
float tone_arr[N_SAMPLES * 2];
int txBuffer[RX_BUFFER_LEN * 2];

static const char *TAG = "main";

/**
 * Helper function for inverse FFT
 * Assumes FFT is full, and coefficients have been generated
*/
esp_err_t inv_fft(float* fft_arr, int num_samples)
{
    // Invert all imaginary values by multiplying by -1
    // This entails all odd entries
    for (int i = 0; i < num_samples; i++)
    {
        fft_arr[2 * i + 1] *= -1;
    }

    ESP_ERROR_CHECK(dsps_fft2r_fc32(fft_arr, num_samples)); 
    ESP_ERROR_CHECK(dsps_bit_rev_fc32(fft_arr, num_samples)); 

    // Correct all real values by sample size
    // Change all imaginary components to 0
    for (int i = 0; i < num_samples; i++)
    {
        fft_arr[2 * i] /= (float)num_samples; // Correction factor
        // fft_arr[2 * i] /= hann_win[i]; // Correct for window
        // Conjugate of imaginary component (should be close to 0)
        fft_arr[2 * i + 1] /= (float)num_samples;
        fft_arr[2 * i + 1] *= -1;
    }

    return ESP_OK;
}

void app_main(void)
{
    // Instantiate pointers to tx buffers
    int* txBuffer_overlap = (int *)calloc(RX_BUFFER_LEN, sizeof(int));

    i2s_chan_handle_t aux_handle;
    
    // Init channel
    ESP_LOGI(TAG, "Initializing DAC I2S interface");
    i2s_chan_config_t aux_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_new_channel(&aux_chan_cfg, &aux_handle, NULL);

    // Initialize config
    i2s_std_config_t aux_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLING_FREQ_HZ),
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
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(aux_handle, &aux_cfg));

    // Enable channels
    ESP_ERROR_CHECK(i2s_channel_enable(aux_handle));

    ESP_LOGI(TAG, "Channels initiated! Initializing FFT coefficients");
    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

    // Generate tone array
    // Amplitude is 2^23 for max absolute value of 24-bit integer (doesn't include sign)
    dsps_tone_gen_f32(tone_buffer, N_SAMPLES, 1 << 23, TONE_FREQ_ARR, 0);
    // Pre dot-product with hann window
    for (int i = 0; i < N_SAMPLES; i++) {
        tone_arr[2 * i] = tone_buffer[i] * hann_win[i]; // Real component
        tone_arr[2 * i + 1] = 0; // Imaginary component
    }

    // Set buffers to 0
    memset(rx_FFT, 0.0, sizeof(rx_FFT));
    memset(tx_iFFT, 0.0, sizeof(tx_iFFT));

    // Subscribe to watchdog timer (NULL->current task)
    esp_task_wdt_add(NULL);
    
    // Main loop
    unsigned int loop_count = 0;
    float fft_calc_time_sum = 0;
    while (1) {
        size_t bytes_read = 0, bytes_written = 0;
        esp_err_t ret_val;

        ////// FFT CALCULATION BEGIN ///////
        unsigned int start_cc = dsp_get_cpu_cycle_count();

        // Copy values of tone tone array into FFT buffer
        memcpy(rx_FFT, tone_arr, sizeof(rx_FFT));

        // FFT Calculation
        dsps_fft2r_fc32(rx_FFT, N);
        // Reverse bits
        dsps_bit_rev_fc32(rx_FFT, N);

        // Copy into iFFT buffer in lieu of element-wise calculations
        // Do manually for proper simulation
        for (int i = 0; i < N_SAMPLES * 2; i++)
        {
            tx_iFFT[i] = rx_FFT[i];
        }
        // Perform iFFT calc
        inv_fft(tx_iFFT, N);
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();
        ////// FFT CALCULATION END ///////

        for (int i = 0; i < RX_BUFFER_LEN; i++)
        {
            // Add-overlay beginning portion of iFFT into txBuffer
            int tx_val = tx_iFFT[2 * i];
            txBuffer[2 * i] = (txBuffer_overlap[i] + tx_val) * 256; // Max out int value
            txBuffer[2 * i + 1] = txBuffer[2 * i]; // Copy L and R

            // Store latter portion for use next loop
            int tx_overlap_val = tx_iFFT[2 * (i + RX_BUFFER_LEN)];
            txBuffer_overlap[i] = tx_overlap_val;
        }
        

        ret_val = i2s_channel_write(aux_handle, &txBuffer, bytes_read, &bytes_written, 5000);
        if (ret_val != ESP_OK || bytes_written != bytes_read) {
            ESP_LOGW(TAG, "Write failed! Err code %d, %d bytes written", (int)ret_val, bytes_written);
        }

        // Calculate time spent, add to running sum
        fft_calc_time_sum += (float)(end_cc - start_cc) / 240e3;
        loop_count++;

        // Spit out average if sum has hit count
        if (loop_count % SAMPLES_PER_AVG == 0)
        {
            // Calculate average time spent calculating FFT/iFFT in ms
            float fft_calc_time_avg = fft_calc_time_sum / SAMPLES_PER_AVG;
            // Calculate average error per element of most recent sample
            float avg_element_err_pct = 0;
            float rx_val, tx_val;
            for (int i = 0; i < N_SAMPLES; i++)
            {
                rx_val = tone_arr[2 * i];
                tx_val = tx_iFFT[2 * i];
                avg_element_err_pct += (rx_val != 0) ? fabsf(tx_val - rx_val) / rx_val :
                                       (tx_val == 0) ? 0 : 1.0;
            }
            avg_element_err_pct /= N_SAMPLES;
            avg_element_err_pct *= 100.0;
            
            ESP_LOGI(TAG, "Loop %i average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
                loop_count, fft_calc_time_avg, avg_element_err_pct);
            // Clear sum
            fft_calc_time_sum = 0;
        }

        // Reset watchdog timeout
        ESP_ERROR_CHECK(esp_task_wdt_reset());
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
