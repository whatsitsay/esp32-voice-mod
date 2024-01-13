/*
 * Microphone test for INMP441 breakout board
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_dsp.h"

// Local libraries
#include <es8388.h>
#include <algo_common.h>
#include <peak_shift.h>
#include <filters.h>

// Plot macros
#define PLOT_LEN (128)

// Number of samples
#define HOP_SIZE (N_SAMPLES / 2) // No overlap here, just data gathering
#define HOP_BUFFER_SIZE_B (2 * HOP_SIZE * 4) // == length of rx/tx buffers (bytes)
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// Local macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs

i2s_chan_handle_t rx_handle, tx_handle;

// Allocate buffer
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define RX_BUFFER_LEN (N_SAMPLES) // Needs to be power of 2 for DSP lib
static int rxBuffer[RX_BUFFER_LEN * 2]; // x2 for L + R channels
static int txBuffer[100];
static unsigned int rx_ovfl_hit = 0;
static unsigned int full_data_rcvd = 0;

// FFT buffers
__attribute__((aligned(16))) float rx_FFT[RX_BUFFER_LEN * 2]; // Will be complex

__attribute__((aligned(16))) float prev_rx_FFT[2 * FFT_MOD_SIZE]; // Needed for instantaneous angle calc
__attribute__((aligned(16))) float rx_FFT_mag[FFT_MOD_SIZE]; // Needed for peak shifting
__attribute__((aligned(16))) float run_phase_comp[2 * FFT_MOD_SIZE]; // Cumulative phase compensation buffer
__attribute__((aligned(16))) float rx_env[FFT_MOD_SIZE];

// RX task attributes
static TaskHandle_t xRxTaskHandle, xTxTaskHandle;
static SemaphoreHandle_t RxBufMutex;
#define RX_TASK_STACK_SIZE (4096u) // Check watermark!
#define RX_TASK_CORE (1)
#define RX_TASK_PRIORITY (10U)
#define TX_TASK_STACK_SIZE (4096u) // Check watermark!
#define TX_TASK_CORE (1)
#define TX_TASK_PRIORITY (10U) // Higher due to it being blocked

/**
 * @brief Callback for RX received event. Will send notification
 * to I2S receive task to pop data.
 * 
 * @param handle I2S channel handle (RX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool rx_rcvd_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Set task notification for RX task to continue
    configASSERT( xRxTaskHandle != NULL );
    vTaskNotifyGiveFromISR(xRxTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    
    return false;
}

/**
 * @brief Callback for RX receive queue overflow. Increments running counter.
 * 
 * @param handle I2S channel handle (TX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool rx_rcvd_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    rx_ovfl_hit++;
    
    return false;
}

void i2s_receive(void* pvParameters)
{
    static const char *TAG = "I2S receive";

    int* stream_data = (int *)pvParameters;

    // Enable RX channel
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    while (1) {
        size_t data_received = 0;

        // Wait for sent notification
        (void) ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS);

        // Read from I2S buffer
        i2s_channel_read(rx_handle, stream_data, sizeof(rxBuffer), &data_received, 500 / portTICK_PERIOD_MS);
        if (data_received != sizeof(rxBuffer)) {
            ESP_LOGE(TAG, "Only read %d out of %d bytes from I2S buffer", data_received, sizeof(rxBuffer));
            if (data_received == 0) continue;
        }
        else
        {
            full_data_rcvd++;
        }

        // Get semaphore
        if (xSemaphoreTake(RxBufMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to get mic buffer mutex");
        }
        // Copy buffer into rxBuffer
        memcpy(rxBuffer, stream_data, data_received);

        // Release semaphore
        xSemaphoreGive(RxBufMutex);
    }
}

// Dummy function for full use of buffer
void i2s_transmit(void* pvParameters)
{
    static const char *TAG = "I2S transmit";

    configASSERT( tx_handle != NULL );

    // Zero-out stream data
    memset(txBuffer, 0, sizeof(txBuffer));
    
    // Enable TX channel
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    while (1) {
        size_t data_written = 0;

        int* stream_data = txBuffer;

        // Write to I2S buffer
        // No error check here to allow for partial writes
        i2s_channel_write(tx_handle, stream_data, sizeof(txBuffer), &data_written, 300 / portTICK_PERIOD_MS);
        if (data_written != sizeof(txBuffer)) {
            ESP_LOGE(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, sizeof(txBuffer));
        }
    }
}

void app_main(void)
{
    static const char *TAG = "main";
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);
    // Set ADC volume to 0 dB for full sensitivity
    es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);
    // Set DAC volume to minimum (-96) to effectively mute
    es8388_set_adc_dac_volume(ES_MODULE_DAC, -96, 0);

    // Instantiate I2S callbacks
    i2s_event_callbacks_t cbs = {
        .on_recv = rx_rcvd_callback,
        .on_recv_q_ovf = rx_rcvd_overflow,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &cbs, NULL));

    // Initialize FFT coefficients
    dsps_fft2r_init_fc32(NULL, N);

    // Set peak shift algorithm config
    peak_shift_cfg_t cfg = {
        .hop_size    = HOP_SIZE,
        .bin_freq_step = 1.0 * I2S_SAMPLING_FREQ_HZ / N_SAMPLES,
        .fft_ptr = rx_FFT,
        .fft_prev_ptr = prev_rx_FFT,
        .fft_mag_ptr = rx_FFT_mag,
        .fft_out_ptr = NULL,
    };
    init_peak_shift_cfg(&cfg);

    // Reset phase compensation buffer
    reset_phase_comp_arr(run_phase_comp);

    // Set true envelope calculation config
    // Use rx_FFT as buffer for both FFT and cepstrum (half the size)
    config_true_env_calc(rx_FFT, rx_FFT + N_SAMPLES, I2S_SAMPLING_FREQ_HZ);

    // Create mutex for debug buffers
    RxBufMutex = xSemaphoreCreateMutex();

    int* stream_data = (int *)malloc(sizeof(rxBuffer));

    // Create RX task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        i2s_receive, 
        "I2S Receive Task",
        RX_TASK_STACK_SIZE,
        (void *)stream_data,
        RX_TASK_PRIORITY,
        &xRxTaskHandle,
        RX_TASK_CORE
    );

    // Create TX task
    memset(txBuffer, 0, sizeof(txBuffer));
    xReturned |= xTaskCreatePinnedToCore(
        i2s_transmit, 
        "I2S Transmit Task",
        TX_TASK_STACK_SIZE,
        NULL,
        TX_TASK_PRIORITY,
        &xTxTaskHandle,
        TX_TASK_CORE
    );

    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create tasks! Error: %d. Restarting ESP in 5 seconds...", xReturned);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    
    // Main loop
    while (1) {
        // Delay every second or so
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Copy to previous FFT array
        memcpy(prev_rx_FFT, rx_FFT, sizeof(prev_rx_FFT));

        // Grab mutex
        if (xSemaphoreTake(RxBufMutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to get mic buffer mutex");
            return;
        }

        // Fill FFT buffs with last sample
        for (int i = 0; i < RX_BUFFER_LEN; i++) {
            // Cast to signed int
            // Skip every other, as it should only be L channel with data
            float rx_f = (float)(rxBuffer[2 * i] >> I2S_DOWNSHIFT);
            // Convert to voltage
            rx_f *= I2S_VOLTAGE_CONV;
            
            // Dot-multiply with Hann windows to reduce effect of edges
            rx_FFT[2 * i] = rx_f * get_window(i);
            rx_FFT[2 * i + 1] = 0; // No complex component
        }

        // Give back mutex
        xSemaphoreGive(RxBufMutex);

        // Calculate FFT, profile cycle count
        unsigned int start_b = dsp_get_cpu_cycle_count();
        ESP_ERROR_CHECK(calc_fft(rx_FFT, RX_BUFFER_LEN));
        // Calculate magnitude
        calc_fft_mag_db(rx_FFT, rx_FFT_mag, FFT_MOD_SIZE);
        // Find local peaks
        find_local_peaks();
        // Convert mag to pure log by dividing all entries by 10
        dsps_mulc_f32(rx_FFT_mag, rx_FFT_mag, FFT_MOD_SIZE, 0.1, 1, 1);

        // Get fundamental frequency estimate from peak shifter module
        volatile float fundamental_freq_est = est_fundamental_freq();
        // Calculate cepstrum
        calc_true_envelope(rx_FFT_mag, rx_env, 440);
        // Convert to dB
        dsps_mulc_f32(rx_FFT_mag, rx_FFT_mag, FFT_MOD_SIZE, 10, 1, 1);
        dsps_mulc_f32(rx_env, rx_env, FFT_MOD_SIZE, 10, 1, 1);

        unsigned int end_b = dsp_get_cpu_cycle_count();
        float fft_comp_time_ms = (float)(end_b - start_b)/240e3;
        
        // Show results
        ESP_LOGW(TAG, "\nMic spectra magnitude (dB)");
        dsps_view(rx_FFT_mag, 1024, 128, 10, -20, 30, 'x');
        ESP_LOGW(TAG, "Mic cepstrum (dB)");
        dsps_view(rx_env, 1024, 128, 10, -20, 30, 'o');
        ESP_LOGI(TAG, "Calc for envelope took %i cycles (%.3f ms)", end_b - start_b, fft_comp_time_ms);
        ESP_LOGI(TAG, "RX overflow count = %d, all data received count = %d", rx_ovfl_hit, full_data_rcvd);
        // Print local peaks
        print_local_peaks();
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
