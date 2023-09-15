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
#define TONE_FREQ_HZ (I2S_SAMPLING_FREQ_HZ / SAMPLES_PER_CYCLE / 2) // A little backwards, but should help even wave
#define TONE_FREQ_SIN (1.0 * TONE_FREQ_HZ / I2S_SAMPLING_FREQ_HZ) // Sinusoid apparent freq

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

// Avg sum values
static unsigned int loop_count = 0;
static float fft_calc_time_sum = 0;

// Instantiate pointers to TX buffers 
float txBuffer_overlap[TX_BUFFER_LEN];
int txBuffer[TX_BUFFER_LEN * 2]; // L + R

// Stream handles
i2s_chan_handle_t rx_handle, tx_handle;
StreamBufferHandle_t xTxStreamBuffer;
#define STREAM_BUFFER_SIZE_B (2 * TX_BUFFER_LEN * 4)

// Task handles
static TaskHandle_t xDSPTaskHandle, xFillerTaskHandle;
const UBaseType_t xFillDoneIdx = 0UL; // For task notifications
#define DSP_TASK_STACK_SIZE (16384u) // Check watermark!
#define DSP_TASK_CORE (0)
#define FILLER_TASK_STACK_SIZE (4096u) // Check watermark!
#define FILLER_TASK_CORE (1)

static SemaphoreHandle_t xTxBufferMutex;

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

void print_task_stats()
{
    const char* TAG = "Task Stats";
    // float* rx_plot = (float *)calloc(PLOT_LEN, sizeof(float));
    // float* tx_plot = (float *)calloc(PLOT_LEN, sizeof(float));

    // Grab mutex
    if (xSemaphoreTake(xTxBufferMutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get TX buffer mutex for stats");
    }

    // Calculate average time spent calculating FFT/iFFT in ms
    float fft_calc_time_avg = fft_calc_time_sum / loop_count;
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
    
    ESP_LOGW(TAG, "Running average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
        fft_calc_time_avg, avg_element_err_pct);

    // Clear sum and count
    fft_calc_time_sum = 0;
    loop_count = 0;

    // // Copy buffers into plot buffers
    // memcpy(rx_plot, rx_dbg, PLOT_LEN * sizeof(float));
    // memcpy(tx_plot, tx_dbg, PLOT_LEN * sizeof(float));

    // Release mutex, as prints take longer
    xSemaphoreGive(xTxBufferMutex);

    // // Print plots
    // ESP_LOGI(TAG, "Input signal:");
    // dsps_view(rx_plot, PLOT_LEN, PLOT_LEN / 2, 15, -1 * TONE_AMPL, TONE_AMPL, '*');
    // ESP_LOGI(TAG, "Output signal:");
    // dsps_view(tx_plot, PLOT_LEN, PLOT_LEN / 2, 15, -1 * TONE_AMPL, TONE_AMPL, '*');

    // // Free memory
    // free(rx_plot);
    // free(tx_plot);
}

///// CALLBACKS ///// 

// Called whenever buffer for output has been emptied
static IRAM_ATTR bool aux_tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Set task notification for filler task to continue
    configASSERT( xFillerTaskHandle != NULL );
    vTaskNotifyGiveIndexedFromISR(xFillerTaskHandle, xFillDoneIdx, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    return false; // FIXME what does this boolean indicate? Docs not clear
}

///// MAIN TASKS ///// 

void proc_audio_data(void* pvParameters)
{
    static const char *TAG = "dsp_loop";

    // Subscribe to watchdog timer (NULL->current task)
    esp_task_wdt_add(NULL);

    // Write 0's to buffer, such that callback is effectively triggered
    size_t bytes_written = 0;
    ESP_ERROR_CHECK(i2s_channel_write(tx_handle, txBuffer, sizeof(txBuffer), &bytes_written, portMAX_DELAY));
    
    // Main loop
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

        // Get mutex. Don't wait long, as these calculations are more crucial
        if (xSemaphoreTake(xTxBufferMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to get TX buffer mutex in time!");
        }

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

        xSemaphoreGive(xTxBufferMutex);
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();

        // Calculate time spent, add to running sum
        fft_calc_time_sum += (float)(end_cc - start_cc) / 240e3;
        loop_count++;


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

void i2s_buffer_filler(void* pvParameters)
{
    size_t data_popped, data_written;
    static const char *TAG = "I2S buffer filler";
    int* stream_data = (int *)calloc(I2S_POP_SIZE, sizeof(int));

    while (1) {
        // Pop data from stream
        data_popped = xStreamBufferReceive(xTxStreamBuffer, stream_data, I2S_POP_SIZE_B, portMAX_DELAY);
        if (data_popped != I2S_POP_SIZE_B) {
            ESP_LOGW(TAG, "Only popped %d out of %d bytes from stream", data_popped, I2S_POP_SIZE_B);
            if (data_popped == 0) continue;
        }

        // Write to I2S buffer
        ESP_ERROR_CHECK(i2s_channel_write(tx_handle, stream_data, data_popped, &data_written, 1000));
        if (data_written != data_popped) {
            ESP_LOGW(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, data_popped);
        }

        // Wait for task notification from callback
        (void) ulTaskNotifyTakeIndexed(xFillDoneIdx, pdTRUE, portMAX_DELAY);
    }
    free(stream_data);
}

void app_main(void)
{

    static const char *TAG = "main";
    
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);

    ESP_LOGW(TAG, "Channel initiated!");

    // Instantiate stream buffer
    xTxStreamBuffer = xStreamBufferCreate(STREAM_BUFFER_SIZE_B, I2S_POP_SIZE_B);
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

    // Create mutex for buffer
    xTxBufferMutex = xSemaphoreCreateMutex();

    // Start I2S Buffer Filler task
    ESP_LOGW(TAG, "Starting Filler task...");
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        i2s_buffer_filler, 
        "I2S Filler Task",
        FILLER_TASK_STACK_SIZE,
        NULL,
        10, // Can be higher, since it should spend most time blocked
        &xFillerTaskHandle,
        FILLER_TASK_CORE
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

    // Instantiate I2S callback
    i2s_event_callbacks_t cbs = {
        .on_recv = NULL,
        .on_recv_q_ovf = NULL,
        .on_sent = aux_tx_sent_callback,
        .on_send_q_ovf = NULL, // May be necessary
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &cbs, NULL));

    // Start DSP task
    ESP_LOGW(TAG, "Starting DSP task...");
    xReturned = xTaskCreatePinnedToCore(
        proc_audio_data, 
        "Audio DSP",
        DSP_TASK_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY, // Verify this priority, should be low
        &xDSPTaskHandle,
        DSP_TASK_CORE
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

    // Enable channels
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    
    // Main loop
    ESP_LOGI(TAG, "Finished setup, entering main loop.");
    while (1) {
        // Delay some time
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        print_task_stats();
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
