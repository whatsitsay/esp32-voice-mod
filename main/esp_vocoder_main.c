/**
 * @file esp_vocoder_main.c
 * @author Gabriel Kaufman (gkaufman93@gmail.com)
 * @brief Main module for ESP32 phase vocoder
 * 
 * Currently capable of the following vocal effects:
 * 1) Pitch-shifted chorus
 * 
 * @version 0.2
 * @date 2023-10-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"
#include "esp_timer.h"

// Local libraries
#include <es8388.h>
#include <algo_common.h>
#include <peak_shift.h>
#include <gpio_button.h>
#include "esp_vocoder_main.h"

////// HELPER FUNCTIONS //////
void print_task_stats(void* pvParameters)
{
    const char* TAG = "Task Stats";
    float* rx_FFT_mag_cpy = (float *)malloc(PLOT_LEN * sizeof(float));
    float* tx_FFT_mag_cpy = (float *)malloc(PLOT_LEN * sizeof(float));

    configASSERT( (rx_FFT_mag_cpy != NULL) && (tx_FFT_mag != NULL) );

    while (1) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Grab mutex
        if (xSemaphoreTake(xDbgMutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to get debug buffer mutex for stats");
            return;
        }

        // Calculate average time spent doing DSP calculations in ms
        float dsp_calc_time_avg = dsp_calc_time_sum / loop_count;
        // Calculate average number of peaks "detected" by algorithm
        float num_peaks_avg = num_peaks_sum / loop_count;
        
        ESP_LOGI(TAG, "Running average DSP calc time: %.4f ms, avg num peaks: %.2f",
            dsp_calc_time_avg, num_peaks_avg);
        ESP_LOGI(TAG, "TX overflow hit count: %0d, RX overflow hit count: %0d", tx_ovfl_hit, rx_ovfl_hit);

        // Clear sum and count
        dsp_calc_time_sum = 0;
        loop_count = 0;
        num_peaks_sum = 0;

        // Copy magnitude buffers
        memcpy(rx_FFT_mag_cpy, rx_FFT_mag, PLOT_LEN * sizeof(float));
        memcpy(tx_FFT_mag_cpy, tx_FFT_mag, PLOT_LEN * sizeof(float));

        // Print local peaks
        print_local_peaks();

        // Release mutex
        xSemaphoreGive(xDbgMutex);

        // Plot magnitudes
        ESP_LOGI(TAG, "Input FT magnitude (dB):");
        dsps_view(rx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, 0, 40, 'x');
        ESP_LOGI(TAG, "Output FT magnitude (dB):");
        dsps_view(tx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, 0, 40, 'o');
        ESP_LOGW(TAG, "Current vocoder mode: %0d", (int)_vocoder_mode);
    }
}

/**
 * @brief Function for performing audio data modification
 * 
 * @param rxBuffer - Buffer containing microphone input sound data
 * @param txBuffer - Output buffer for modified sound data for transmission
 */
void audio_data_modification(int* rxBuffer, int* txBuffer) {
    static const char* TAG = "Audio Modification";

    // Copy RX overlap into debug buffer
    if (xSemaphoreTake(xDbgMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get mutex for RX debug buffer!");
    }

    // Copy former half of current rx_FFT into previous FFT buffer
    // Use sizeof previous buffer, given it should be smaller (symmetrical data)
    memcpy(prev_rx_FFT, rx_FFT, sizeof(prev_rx_FFT));

    // Copy values of rxBuffer into FFT buffer
    for (int i = 0; i < N_SAMPLES; i++) {
        // Select sample. If first half, use rxBuffer_overlap. Otherwise use rxBuffer
        int rx_val = (i < HOP_SIZE) ? rxBuffer_overlap[i] : rxBuffer[2 * (i - HOP_SIZE)];

        // Dot-product with hann window
        // Downshift to prevent overflow (last 8 bits are always 0)
        rx_FFT[2 * i] = (float)(rx_val >> I2S_DOWNSHIFT) * get_window(i);
        // Set imaginary component to 0
        rx_FFT[2 * i + 1] = 0;
    }

    // FFT Calculation
    ESP_ERROR_CHECK(calc_fft(rx_FFT, N));

    // Find peaks
    int num_peaks = find_local_peaks();
    configASSERT( num_peaks >= 0 ); // If -1, reached max
    num_peaks_sum += num_peaks;

    // Copy in tonic to start
    memcpy(tx_iFFT, rx_FFT, FFT_MOD_SIZE * 2 * sizeof(float));

    if (num_peaks > 0) {
        // Perform peak shift, if there are any peaks
        for (int i = 0; i < NUM_PITCH_SHIFTS; i++) {
            shift_peaks_int(PITCH_SHIFT_FACTORS[i], PITCH_SHIFT_GAINS[i], run_phase_comp);
        }
    } else {
        // Otherwise reset phase compensation array
        reset_phase_comp_arr(run_phase_comp);
    }

    // Calculate magnitudes
    calc_fft_mag_db(tx_iFFT, tx_FFT_mag, PLOT_LEN);

    // Fill latter half of FFT with conjugate mirror data
    fill_mirror_fft(tx_iFFT, N);

    // Perform iFFT calc
    ESP_ERROR_CHECK(inv_fft(tx_iFFT, N));

    // Fill TX buffer
    for (int i = 0; i < HOP_SIZE; i++)
    {
        // Add-overlay beginning portion of iFFT into txBuffer
        float tx_val = tx_iFFT[2 * i] * get_window(i); // Window result
        txBuffer[2 * i] = (int)(txBuffer_overlap[i] + tx_val); 
        txBuffer[2 * i] <<= I2S_DOWNSHIFT; // Increase int value
        txBuffer[2 * i + 1] = txBuffer[2 * i]; // Copy L and R

        // Store latter portion for use next loop
        float tx_overlap_val = tx_iFFT[2 * (i + HOP_SIZE)] * get_window(i + HOP_SIZE);
        txBuffer_overlap[i] = tx_overlap_val;
    }
    xSemaphoreGive(xDbgMutex);
}

///// CALLBACKS ///// 

/**
 * @brief Callback for TX send completion. Will send notification to I2S
 * transmit task to push data.
 * 
 * @param handle I2S channel handle (TX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Set task notification for filler task to continue
    configASSERT( xTxTaskHandle != NULL );
    vTaskNotifyGiveFromISR(xTxTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    return false; // FIXME what does this boolean indicate? Docs not clear
}

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

/**
 * @brief Callback for TX send queue overflow. Increments running counter.
 * 
 * @param handle I2S channel handle (TX in this case)
 * @param event Event data
 * @param user_ctx User data
 * @return IRAM_ATTR False
 */
static IRAM_ATTR bool tx_sent_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    tx_ovfl_hit++;
    
    return false;
}

static IRAM_ATTR void mode_switch_cb(void* args)
{
    // Check value
    int read_val = gpio_get_level(MODE_SWITCH_PIN);

    // Only give mode switch semaphore if value is low
    if (!read_val) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xModeSwitchSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    // Re-enable GPIO pin interrupt
    gpio_intr_enable(MODE_SWITCH_PIN);
}

///// TASKS ///// 

void proc_audio_data(void* pvParameters)
{
    // Main loop
    while (1) {
        // Clear event bit
        (void) xEventGroupClearBits(xTaskSyncBits, DSP_TASK_BIT);

        // Set buffer pointers
        int* rxBuffer = rxBuffers[dsp_idx];
        int* txBuffer = txBuffers[dsp_idx];

        unsigned int start_cc = dsp_get_cpu_cycle_count();

        // Modify data
        audio_data_modification(rxBuffer, txBuffer);
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();

        // Calculate time spent, add to running sum
        dsp_calc_time_sum += (float)(end_cc - start_cc) / 240e3;
        loop_count++;

        // Copy rxBuffer into rxBuffer_overlap
        for (int i = 0; i < HOP_SIZE; i++)
        {
            rxBuffer_overlap[i] = rxBuffer[2 * i];
        }

        // Set event group bit
        // Ignore result (will be checked in main loop)
        (void) xEventGroupSync(xTaskSyncBits,
                               DSP_TASK_BIT,
                               ALL_SYNC_BITS,
                               portMAX_DELAY);
    }
    
}

void i2s_receive(void* pvParameters)
{
    static const char *TAG = "I2S receive";

    configASSERT( rx_handle != NULL );

    // Enable RX channel
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    while (1) {
        size_t data_received = 0;
        
        // Clear event bit
        (void) xEventGroupClearBits(xTaskSyncBits, RX_TASK_BIT);

        // Wait for sent notification
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int* stream_data = rxBuffers[i2s_idx];

        // Read from I2S buffer
        i2s_channel_read(rx_handle, stream_data, HOP_BUFFER_SIZE_B, &data_received, 300 / portTICK_PERIOD_MS);
        if (data_received != HOP_BUFFER_SIZE_B) {
            ESP_LOGE(TAG, "Only read %d out of %d bytes from I2S buffer", data_received, HOP_BUFFER_SIZE_B);
        }

        // Set event group bit
        // Ignore result (will be checked in main loop)
        (void) xEventGroupSync(xTaskSyncBits,
                               RX_TASK_BIT,
                               ALL_SYNC_BITS,
                               portMAX_DELAY);

    }
}

void i2s_transmit(void* pvParameters)
{
    static const char *TAG = "I2S transmit";

    configASSERT( tx_handle != NULL );
    
    // Enable TX channel
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    while (1) {
        size_t data_written = 0;

        // Clear event bits
        (void) xEventGroupClearBits(xTaskSyncBits, TX_TASK_BIT);

        int* stream_data = txBuffers[i2s_idx];

        // Write to I2S buffer
        // No error check here to allow for partial writes
        i2s_channel_write(tx_handle, stream_data, HOP_BUFFER_SIZE_B, &data_written, 300 / portTICK_PERIOD_MS);
        if (data_written != HOP_BUFFER_SIZE_B) {
            ESP_LOGE(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, HOP_BUFFER_SIZE_B);
        }

        // Set event group bit
        // Ignore result (will be checked in main loop)
        (void) xEventGroupSync(xTaskSyncBits,
                               TX_TASK_BIT,
                               ALL_SYNC_BITS,
                               portMAX_DELAY);
    }
}

///// MAIN LOOP ///// 

void app_main(void)
{
    static const char *TAG = "main";
    BaseType_t xReturned = 0UL;
    
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);

    ESP_LOGW(TAG, "Channels initiated!");

    // Instantiate TX/RX buffers
    for (int i = 0; i < NUM_BUFFERS; i++) {
        txBuffers[i] = (int *)calloc(2 * HOP_SIZE, sizeof(int));
        rxBuffers[i] = (int *)calloc(2 * HOP_SIZE, sizeof(int));

        configASSERT( (txBuffers[i] != NULL) && (rxBuffers[i] != NULL) );
    }

    // Clear out all other memory buffers
    memset(rx_FFT, 0, sizeof(rx_FFT));
    memset(tx_iFFT, 0.0, sizeof(tx_iFFT));
    memset(txBuffer_overlap, 0, sizeof(txBuffer_overlap));
    memset(rxBuffer_overlap, 0, sizeof(rxBuffer_overlap));

    // Initialize FFT coefficients
    dsps_fft2r_init_fc32(NULL, N);

    // Set peak shift algorithm config
    peak_shift_cfg_t cfg = {
        .num_samples = N,
        .hop_size    = HOP_SIZE,
        .bin_freq_step = 1.0 * I2S_SAMPLING_FREQ_HZ / N_SAMPLES,
        .fft_ptr = rx_FFT,
        .fft_prev_ptr = prev_rx_FFT,
        .fft_mag_ptr = rx_FFT_mag,
        .fft_out_ptr = tx_iFFT,
    };
    init_peak_shift_cfg(&cfg);

    // Reset phase compensation buffer
    reset_phase_comp_arr(run_phase_comp);

    // Intantiate indices
    i2s_idx = I2S_IDX_START;
    dsp_idx = DSP_IDX_START;

    // Create mutex for debug buffers
    xDbgMutex = xSemaphoreCreateMutex();

    // Create semaphore for mode switch
    xModeSwitchSem = xSemaphoreCreateBinary();

    // Instantiate event group for task sync
    xTaskSyncBits = xEventGroupCreate();

    // Initiate mode switch button
    ESP_ERROR_CHECK(init_gpio_button(MODE_SWITCH_PIN, &mode_switch_cb));

    // Instantiate I2S callbacks
    i2s_event_callbacks_t cbs = {
        .on_recv = rx_rcvd_callback,
        .on_recv_q_ovf = rx_rcvd_overflow,
        .on_sent = tx_sent_callback,
        .on_send_q_ovf = tx_sent_overflow,
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &cbs, NULL));

    // Start DSP task
    ESP_LOGW(TAG, "Starting DSP task...");
    xReturned |= xTaskCreatePinnedToCore(
        proc_audio_data, 
        "Audio DSP",
        DSP_TASK_STACK_SIZE,
        NULL,
        DSP_TASK_PRIORITY, // Verify this priority, should be low
        &xDSPTaskHandle,
        DSP_TASK_CORE
    );

    // Start RX/TX buffer tasks
    ESP_LOGW(TAG, "Starting RX task...");
    xReturned |= xTaskCreatePinnedToCore(
        i2s_receive, 
        "I2S Receive Task",
        RX_TASK_STACK_SIZE,
        NULL,
        RX_TASK_PRIORITY,
        &xRxTaskHandle,
        RX_TASK_CORE
    );
    ESP_LOGW(TAG, "Starting TX task...");
    xReturned |= xTaskCreatePinnedToCore(
        i2s_transmit, 
        "I2S Transmit Task",
        TX_TASK_STACK_SIZE,
        NULL,
        TX_TASK_PRIORITY,
        &xTxTaskHandle,
        TX_TASK_CORE
    );

    ESP_LOGW(TAG, "Starting stats task...");
    xReturned |= xTaskCreatePinnedToCore(
        print_task_stats, 
        "Statistics Task",
        TASK_STATS_STACK_SIZE,
        NULL,
        TASK_STATS_PRIORITY,
        &xTaskStatsHandle,
        TASK_STATS_CORE
    );

    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create tasks! Error: %d. Restarting ESP in 5 seconds...", xReturned);
        vTaskDelete(xDSPTaskHandle);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    
    // Main loop
    ESP_LOGI(TAG, "Finished setup, entering main loop.");
    while (1) {
        EventBits_t uxReturn;
        // Clear switch bit
        (void) xEventGroupClearBits(xTaskSyncBits, BUFF_SWAP_BITS);
        // Wait for all other tasks to finish
        uxReturn = xEventGroupWaitBits(xTaskSyncBits,
                                       BUFF_SWAP_BITS,
                                       pdFALSE, // Don't clear on exit
                                       pdTRUE, // Wait for all bits
                                       SYNC_TIMEOUT_TICKS);

        // Assert all bits have been set
        configASSERT( (uxReturn & BUFF_SWAP_BITS) == BUFF_SWAP_BITS );

        // Swap indices with XOR
        dsp_idx ^= 1;
        i2s_idx ^= 1;

        configASSERT( dsp_idx != i2s_idx );

        // Check headphone jack toggle
        es_toggle_power_amp();

        // Switch mode if button is hit
        if (xSemaphoreTake(xModeSwitchSem, 0) == pdTRUE) {
            _vocoder_mode++;
            _vocoder_mode %= MAX_VOCODER_MODES;
        }

        // Set remaining bit
        uxReturn = xEventGroupSync(xTaskSyncBits,
                                   SWAP_COMPLETE_BIT,
                                   ALL_SYNC_BITS,
                                   0); // Should be no delay
        configASSERT( (uxReturn & ALL_SYNC_BITS) == ALL_SYNC_BITS );
    }

    // If this point is hit, wait some time then restart
    ESP_LOGE(TAG, "Exited main loop! Restarting ESP in 5 seconds...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_restart();
}
