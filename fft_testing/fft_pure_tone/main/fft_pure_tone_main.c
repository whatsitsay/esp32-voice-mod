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
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"

// Local libraries
#include <es8388.h>

// Local macros
#define TONE_BITS (24)
#define TONE_AMPL (powf(2, TONE_BITS - 1) * 0.75) // Less than max 
#define SAMPLES_PER_CYCLE (128) // Even for I2S
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define TONE_FREQ_HZ (I2S_SAMPLING_FREQ_HZ / SAMPLES_PER_CYCLE) // A little backwards, but should help even wave
#define TONE_FREQ_SIN (1.0 * TONE_FREQ_HZ / I2S_SAMPLING_FREQ_HZ) // Sinusoid apparent freq

#define SAMPLES_PER_AVG (100)
#define N_SAMPLES (4096)
#define I2S_POP_SIZE (TX_BUFFER_LEN) // Play around with size
#define I2S_POP_SIZE_B (I2S_POP_SIZE * 4)
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

// Stream handles
i2s_chan_handle_t rx_handle, tx_handle;
StreamBufferHandle_t xTxStreamBuffer;
StaticStreamBuffer_t xTxStreamBufferStruct;
#define STREAM_BUFFER_SIZE_B (2 * TX_BUFFER_LEN * 4)
uint8_t xTxStreamBuffer_storage[STREAM_BUFFER_SIZE_B + 1];
// Task handles
TaskHandle_t xDSPTaskHandle;
#define DSP_TASK_STACK_SIZE (16384u) // Check watermark!

////// HELPER FUNCTIONS //////

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

///// CALLBACKS ///// 

// Called whenever buffer for output has been emptied
// There shoud be data ready from the main task by this point
static IRAM_ATTR bool aux_tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    size_t data_popped, data_written;
    static const char *TAG = "I2S TX sent callback";
    // Pop data from stream
    int* stream_data = (int *)calloc(I2S_POP_SIZE, sizeof(int));
    data_popped = xStreamBufferReceiveFromISR(xTxStreamBuffer, stream_data, I2S_POP_SIZE_B, NULL);
    if (data_popped != I2S_POP_SIZE_B) {
        ESP_LOGW(TAG, "Only popped %d out of %d bytes from stream", data_popped, I2S_POP_SIZE_B);
    }

    ESP_ERROR_CHECK(i2s_channel_write(handle, stream_data, data_popped, &data_written, 1000));
    if (data_written != data_popped) {
        ESP_LOGW(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, data_popped);
    }
    free(stream_data);

    return false; // FIXME what does this boolean indicate? Docs not clear
}

///// MAIN TASK ///// 

void proc_audio_data(void* pvParameters)
{
    static const char *TAG = "dsp_loop";

    // Instantiate pointers to TX buffers 
    float txBuffer_overlap[TX_BUFFER_LEN];
    int txBuffer[TX_BUFFER_LEN * 2]; // L + R

    // Subscribe to watchdog timer (NULL->current task)
    esp_task_wdt_add(NULL);

    // Write 0's to buffer, such that callback is effectively triggered
    size_t bytes_written = 0;
    ESP_ERROR_CHECK(i2s_channel_write(tx_handle, txBuffer, sizeof(txBuffer), &bytes_written, portMAX_DELAY));
    
    // Main loop
    unsigned int loop_count = 0;
    float fft_calc_time_sum = 0;
    while (1) {
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
                float rx_tx_diff_raw = tx_dbg[i] - rx_dbg[i];
                float rx_tx_diff_pct = (rx_dbg[i] != 0) ? rx_tx_diff_raw / fabsf(rx_dbg[i]) :
                                    (tx_dbg[i] == 0) ? 0 : 1.0;
                rx_tx_diff_pct *= 100.0;
                avg_element_err_pct += fabsf(rx_tx_diff_pct);
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

        // Write to stream buffer
        bytes_written = xStreamBufferSend(xTxStreamBuffer, txBuffer, STREAM_BUFFER_SIZE_B, 5000);
        if (bytes_written != STREAM_BUFFER_SIZE_B) {
            ESP_LOGE(TAG, "Failed to write all txBuffer data to stream buffer! %0d B out of %0d B",
                    bytes_written, STREAM_BUFFER_SIZE_B);
        }

        // Reset watchdog timeout
        ESP_ERROR_CHECK(esp_task_wdt_reset());
    }
    
}

void app_main(void)
{

    static const char *TAG = "main";
    
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);

    // Instantiate I2S callback
    i2s_event_callbacks_t cbs = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = aux_tx_sent_callback,
        .on_send_q_ovf = NULL, // May be necessary
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &cbs, NULL));

    // Enable channels
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    ESP_LOGW(TAG, "Channel initiated!");

    // Instantiate stream buffer
    xTxStreamBuffer = xStreamBufferCreateStatic(STREAM_BUFFER_SIZE_B, I2S_POP_SIZE_B, xTxStreamBuffer_storage, &xTxStreamBufferStruct);
    if (xTxStreamBuffer == NULL) 
    {
        ESP_LOGE(TAG, "Failed to instantiate stream buffer!");
    }
    else ESP_LOGW(TAG, "Stream buffer instantiated!");

    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

    // Generate tone array
    dsps_tone_gen_f32(tone_buffer, TONE_SAMPLE_LEN, TONE_AMPL, TONE_FREQ_SIN, 0);

    // Start main task
    ESP_LOGW(TAG, "Starting DSP task...");
    BaseType_t xReturned = xTaskCreate(
        proc_audio_data, 
        "Audio DSP",
        DSP_TASK_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY, // Verify this priority, should be low
        &xDSPTaskHandle
    );
    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create main task! Error: %d. Restarting ESP in 5 seconds...", xReturned);
        vTaskDelete(xDSPTaskHandle);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    else
    {
        ESP_LOGW(TAG, "Instantiated DSP task successfully!");
    }
    
    // Main loop
    ESP_LOGI(TAG, "Finished setup, entering main loop.");
    while (1) {
        // For now, just periodically delay
        // DSP task and ISR's should handle the work
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
