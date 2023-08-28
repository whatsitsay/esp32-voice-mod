/*
 * Microphone test for INMP441 breakout board
 */

#include <stdio.h>
#include <string.h>
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
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs

// Allocate buffer
#define SAMPLES_PER_AVG (100)
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define N_SAMPLES (4096)
#define RX_BUFFER_LEN (N_SAMPLES / 2) // Try bigger spec
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// FFT buffers
__attribute__((aligned(16))) float hann_win[N_SAMPLES];
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Instantiate pointers to debug buffers
float rx_dbg[N_SAMPLES * 2];
float tx_dbg[N_SAMPLES * 2];

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

    ESP_ERROR_CHECK(dsps_fft2r_fc32_ae32(fft_arr, num_samples)); 
    ESP_ERROR_CHECK(dsps_bit_rev_fc32(fft_arr, num_samples)); 

    // Correct all values by sample size and hann window
    for (int i = 0; i < 2 * num_samples; i++)
    {
        fft_arr[i] /= (float)num_samples;
        fft_arr[i] *= hann_win[i/2];
        if (i % 2 == 1) fft_arr[i] *= -1; // Should be ~0, but to be safe
    }

    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing microphone I2S interface...");

    i2s_chan_handle_t mic_handle, aux_handle;
    // Init channel
    i2s_chan_config_t mic_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&mic_chan_cfg, NULL, &mic_handle);
    
    // Instantiate pointers to rx/tx buffers
    int* rxBuffer = (int *)calloc(RX_BUFFER_LEN * 2, sizeof(int));
    int* txBuffer = (int *)calloc(RX_BUFFER_LEN * 2, sizeof(int));
    int* rxBuffer_overlap = (int *)calloc(RX_BUFFER_LEN, sizeof(int));
    int* txBuffer_overlap = (int *)calloc(RX_BUFFER_LEN, sizeof(int));

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

    ESP_LOGI(TAG, "Initializing DAC I2S interface");
    
    // Init channel
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
    ESP_ERROR_CHECK(i2s_channel_enable(mic_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(aux_handle));

    ESP_LOGI(TAG, "Channels initiated! Initializing FFT coefficients");
    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

    // Set buffers to 0
    memset(rx_FFT, 0.0, sizeof(rx_FFT));
    memset(tx_iFFT, 0.0, sizeof(tx_iFFT));
    
    // Main loop
    unsigned int loop_count = 0;
    float fft_calc_time_sum = 0;
    while (1) {
        size_t bytes_read = 0, bytes_written = 0;
        esp_err_t ret_val;

        // Copy rxBuffer into rxBuffer_overlap
        for (int i = 0; i < RX_BUFFER_LEN; i++)
        {
            rxBuffer_overlap[i] = rxBuffer[2 * i];
        }

        // Perform read
        ret_val = i2s_channel_read(mic_handle, &rxBuffer, sizeof(rxBuffer), &bytes_read, 5000);

        if (ret_val != ESP_OK || bytes_read != sizeof(rxBuffer)) {
            ESP_LOGW(TAG, "Read failed! Err code %d, %d bytes read", (int)ret_val, bytes_read);
            continue;
        }

        ////// FFT CALCULATION BEGIN ///////
        unsigned int start_cc = dsp_get_cpu_cycle_count();

        // Copy values of rxBuffer into FFT buffer
        for (int i = 0; i < N_SAMPLES; i++) {
            // Select sample. If first half, use rxBuffer_overlap. Otherwise use rxBuffer
            int rx_val = (i < RX_BUFFER_LEN) ? rxBuffer_overlap[i] : rxBuffer[2 * (i - RX_BUFFER_LEN)];

            // Dot-product with hann window
            // Divide by 256 to prevent overflow (last 8 bits are always 0)
            rx_FFT[2 * i] = (float)(rx_val / 256) * hann_win[i];
            // Set imaginary component to 0
            rx_FFT[2 * i + 1] = 0;
        }

        // Copy to debug buffer
        memcpy(rx_dbg, rx_FFT, sizeof(rx_FFT));

        // FFT Calculation
        dsps_fft2r_fc32_ae32(rx_FFT, N);
        // Reverse bits
        dsps_bit_rev_fc32(rx_FFT, N);

        // Copy into iFFT buffer in lieu of element-wise calculations
        // Do manually for proper simulation
        for (int i = 0; i < N_SAMPLES * 2; i++)
        {
            tx_iFFT[i] = rx_FFT[i];
        }
        // Copy over to debug
        memcpy(tx_dbg, tx_iFFT, sizeof(tx_iFFT));

        // Perform iFFT calc
        inv_fft(tx_iFFT, N);
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();
        ////// FFT CALCULATION END ///////

        for (int i = 0; i < RX_BUFFER_LEN; i++)
        {
            // Add-overlay beginning portion of iFFT into txBuffer
            int tx_val = tx_iFFT[2 * i];
            txBuffer[2 * i] = (txBuffer_overlap[i] + tx_val) * 256; // Re-multiply by 256 for proper alignment
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
            for (int i = 0; i < RX_BUFFER_LEN; i++)
            {
                int rx_val = rxBuffer[2 * i] / 256;
                int tx_val = txBuffer[2 * i] / 256;
                avg_element_err_pct += (rx_val != 0) ? (float)abs(tx_val - rx_val) / (float)rx_val :
                                       (tx_val == 0) ? 0 : 1.0;
            }
            avg_element_err_pct /= RX_BUFFER_LEN;
            avg_element_err_pct *= 100.0;
            
            ESP_LOGI(TAG, "Loop %i average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
                loop_count, fft_calc_time_avg, avg_element_err_pct);
            // Clear sum
            fft_calc_time_sum = 0;
        }
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
