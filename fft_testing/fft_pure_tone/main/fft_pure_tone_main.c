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
#define TONE_BITS (24)
#define TONE_AMPL (powf(2, TONE_BITS - 1) * 0.75) // Less than max 
#define SAMPLES_PER_CYCLE (128) // Even for I2S
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define TONE_FREQ_HZ (I2S_SAMPLING_FREQ_HZ / SAMPLES_PER_CYCLE) // A little backwards, but should help even wave
#define TONE_FREQ_SIN (1.0 * TONE_FREQ_HZ / I2S_SAMPLING_FREQ_HZ) // Sinusoid apparent freq

// Allocate buffer
#define SAMPLES_PER_AVG (499)
#define N_SAMPLES (4096)
#define TX_BUFFER_LEN (N_SAMPLES / 2) // Try bigger spec
int N = N_SAMPLES;

// FFT buffers
__attribute__((aligned(16))) float hann_win[N_SAMPLES];
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Instantiate pointers to debug buffers
#define TONE_SAMPLE_LEN (N_SAMPLES) // Even division
#define PLOT_LEN (2 * SAMPLES_PER_CYCLE)
float tone_buffer[TONE_SAMPLE_LEN];
float rx_dbg[TX_BUFFER_LEN];
float tx_dbg[TX_BUFFER_LEN];
float rx_tx_diff_raw[TX_BUFFER_LEN];
float rx_tx_diff_pct[TX_BUFFER_LEN];
int txBuffer[TX_BUFFER_LEN * 2]; // L + R

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
        // fft_arr[2 * i] *= hann_win[i]; // For proper reconstruction
        // Conjugate of imaginary component (should be close to 0)
        fft_arr[2 * i + 1] /= (float)num_samples;
        fft_arr[2 * i + 1] *= -1;
    }

    return ESP_OK;
}

void app_main(void)
{
    // Instantiate pointers to tx buffers
    float* txBuffer_overlap = (float *)calloc(TX_BUFFER_LEN, sizeof(int));

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

    ESP_LOGW(TAG, "Channels initiated! Initializing FFT coefficients");
    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

    // Generate tone array
    dsps_tone_gen_f32(tone_buffer, TONE_SAMPLE_LEN, TONE_AMPL, TONE_FREQ_SIN, 0);

    // Set buffers to 0
    memset(rx_FFT, 0.0, sizeof(rx_FFT));
    memset(tx_iFFT, 0.0, sizeof(tx_iFFT));

    // Subscribe to watchdog timer (NULL->current task)
    esp_task_wdt_add(NULL);
    
    // Main loop
    unsigned int loop_count = 0;
    float fft_calc_time_sum = 0;
    while (1) {
        size_t bytes_written = 0;
        esp_err_t ret_val;

        unsigned int start_cc = dsp_get_cpu_cycle_count();

        // Copy values manually from tone buffer into FFT and rx_dbg arrays
        for (int i = 0; i < N_SAMPLES; i++) {
            // No need to change index, should be identical each time
            float tone_val = tone_buffer[i];
            rx_FFT[2 * i] = tone_val * hann_win[i]; // Window result
            rx_FFT[2 * i + 1] = 0; // No imaginary component, real signal

            // Copy to debug buffer for first half
            // Floor function to match tx_dbg
            if (i < TX_BUFFER_LEN) rx_dbg[i] = tone_val;
        }


        ////// FFT CALCULATION BEGIN ///////

        // // FFT Calculation
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

        ////// FFT CALCULATION END ///////

        for (int i = 0; i < TX_BUFFER_LEN; i++)
        {
            // Add-overlay beginning portion of iFFT into txBuffer
            float tx_val = tx_iFFT[2 * i];
            txBuffer[2 * i] = (int)(txBuffer_overlap[i] + tx_val); 
            txBuffer[2 * i] <<= (32 - TONE_BITS); // Increase int value
            txBuffer[2 * i + 1] = txBuffer[2 * i]; // Copy L and R

            // Store TX val to debug array for later comparison
            tx_dbg[i] = txBuffer_overlap[i] + tx_val;

            // Store latter portion for use next loop
            float tx_overlap_val = tx_iFFT[2 * (i + TX_BUFFER_LEN)];
            txBuffer_overlap[i] = tx_overlap_val;
        }
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();

        ret_val = i2s_channel_write(aux_handle, txBuffer, TX_BUFFER_LEN, &bytes_written, portMAX_DELAY);
        if (ret_val != ESP_OK || bytes_written != TX_BUFFER_LEN) {
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
            for (int i = 0; i < TX_BUFFER_LEN; i++)
            {
                rx_tx_diff_raw[i] = tx_dbg[i] - rx_dbg[i];
                rx_tx_diff_pct[i] = (rx_dbg[i] != 0) ? rx_tx_diff_raw[i] / fabsf(rx_dbg[i]) :
                                    (tx_dbg[i] == 0) ? 0 : 1.0;
                rx_tx_diff_pct[i] *= 100.0;
                avg_element_err_pct += fabsf(rx_tx_diff_pct[i]);
            }
            avg_element_err_pct /= TX_BUFFER_LEN;
            
            ESP_LOGW(TAG, "Loop %i average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
                loop_count, fft_calc_time_avg, avg_element_err_pct);
            ESP_LOGI(TAG, "Input signal:");
            dsps_view(rx_dbg, PLOT_LEN, PLOT_LEN / 2, 15, -1 * TONE_AMPL, TONE_AMPL, '*');
            ESP_LOGI(TAG, "Output signal:");
            dsps_view(tx_dbg, PLOT_LEN, PLOT_LEN / 2, 15, -1 * TONE_AMPL, TONE_AMPL, '*');
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
