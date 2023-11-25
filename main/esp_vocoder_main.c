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
#include "esp_debug_helpers.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"

// Local libraries
#include <es8388.h>
#include <algo_common.h>
#include <peak_shift.h>
#include <gpio_button.h>
#include "esp_vocoder_main.h"

////// HELPER FUNCTIONS //////

/**
 * @brief Function for performing audio data modification
 * 
 * @param rxBuffer - Buffer containing microphone input sound data
 * @param txBuffer - Output buffer for modified sound data for transmission
 */
void audio_data_modification(int* rxBuffer, int* txBuffer) {
    static const char* TAG = "Audio Modification";
    static vocoder_mode_e prev_vocoder_mode = MOD_CHORUS;
    static unsigned silence_count = 0;

    // Copy RX overlap into debug buffer
    if (xSemaphoreTake(xDbgMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get mutex for RX debug buffer!");
    }

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

    // Store previous frame phase
    memcpy(prev_rx_phase, rx_FFT_phase, sizeof(prev_rx_phase));

    // Magnitude and phase calculations
    float max_mag_db = calc_fft_mag_db(rx_FFT, rx_FFT_mag, FFT_MOD_SIZE);
    calc_fft_phase(rx_FFT, rx_FFT_phase, FFT_MOD_SIZE);

    // Find peaks
    int num_peaks = find_local_peaks();

    num_peaks_sum += num_peaks;

    // Perform peak shift, if there are any peaks
    // First get mode mutex
    xSemaphoreTake(xModeSwitchMutex, 10 / portTICK_PERIOD_MS); // Wait very little time to prevent latency

    // Conditions for resetting phase
    if (prev_vocoder_mode != vocoder_mode) {
        // Reset phase compensation array on mode switch
        reset_phase_comp_arr(run_phase_comp);
        // Reset silence count
        silence_count = 0;
    } else if (max_mag_db < NOISE_THRESHOLD_DB) {
        // Reset array if near silence (below threshold) base on counter
        silence_count++;
        if (silence_count % SILENCE_RESET_COUNT) {
            reset_phase_comp_arr(run_phase_comp);
            phase_reset_count++;
        }
    } else {
        // Reset silence count
        silence_count = 0;
    }
    // Otherwise, leave running

    switch (vocoder_mode) {
        case MOD_CHORUS: {
            // Perform full chorus shift
            for (int i = 0; i < NUM_PITCH_SHIFTS; i++) {
                shift_peaks(PITCH_SHIFT_FACTORS[i], PITCH_SHIFT_GAINS[i], run_phase_comp);
            }
            break;
        }
        case MOD_LOW: {
            shift_peaks(LOW_EFFECT_SHIFT, LOW_EFFECT_GAIN, run_phase_comp);
            break;
        }
        case MOD_HIGH: {
            shift_peaks(HIGH_EFFECT_SHIFT, HIGH_EFFECT_GAIN, run_phase_comp);
            break;
        }
        case PASSTHROUGH: {
            // NOTE: this could just be another copy situation
            shift_peaks(PASSTHROUGH_SHIFT, PASSTHROUGH_GAIN, run_phase_comp);
            break;
        }
        default: {
            // Copy in original sound, reset running phase comp
            memcpy(tx_iFFT, rx_FFT, FFT_MOD_SIZE * 2 * sizeof(float));
            reset_phase_comp_arr(run_phase_comp);
            phase_reset_count++;
        }
    }
    // Save previous mode
    prev_vocoder_mode = vocoder_mode;
    // Give mutex
    xSemaphoreGive(xModeSwitchMutex);

    // Calculate output magnitudes
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

void set_mode_leds()
{
    int mode_int = (int)vocoder_mode;
    // Set to the opposite mode of each bit, as they need to be pulled down
    gpio_set_level(GPIO_LED_1, (~mode_int) & 0x1);
    gpio_set_level(GPIO_LED_2, ~(mode_int >> 1) & 0x1);
}

///// CALLBACKS ///// 

#if configCHECK_FOR_STACK_OVERFLOW == 1
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
    size_t data_written;
    // Flush I2S transmit by sending all 0s
    memset(txBuffers[i2s_idx], 0, HOP_BUFFER_SIZE_B);
    i2s_channel_write(tx_handle, stream_data, HOP_BUFFER_SIZE_B, &data_written, 1000 / portTICK_PERIOD_MS);
    // Restart ESP32
    esp_restart();
}
#endif

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

///// TASKS ///// 

void proc_audio_data(void* pvParameters)
{
    static const char* TAG = "Audio DSP";
    // Wait for initial notification from RX task, so first audio data is half-valid
    // This should limit delay to just the first swap, i.e. 50 ms
    (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
        volatile float iter_calc_time = (float)(end_cc - start_cc) / 240e3;
        // Check against buffering time for I2S, with a little extra in case
        // If this is failed, then the audio will be choppy
        if (iter_calc_time > 1.1 * I2S_BUFFER_TIME_MS) {
            ESP_LOGE(TAG, "DSP calculation time too long! Should be <= %.1f ms, is instead %.3f", I2S_BUFFER_TIME_MS, iter_calc_time);
            configASSERT(false);
        }

        dsp_calc_time_sum += iter_calc_time;
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

    size_t data_received = 0;

    configASSERT( rx_handle != NULL );

    // Enable RX channel
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    // Perform first read with dsp buffer, so that it has real data to use
    // for the first frame
    i2s_channel_read(rx_handle, rxBuffers[dsp_idx], HOP_BUFFER_SIZE_B, &data_received, 300 / portTICK_PERIOD_MS);
    if (data_received != HOP_BUFFER_SIZE_B) {
        ESP_LOGE(TAG, "Only read %d out of %d bytes from I2S buffer", data_received, HOP_BUFFER_SIZE_B);
    }

    // Notify TX and DSP task to continue on
    xTaskNotifyGive(xTxTaskHandle);
    xTaskNotifyGive(xDSPTaskHandle);

    while (1) {
        
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

    // Wait for initial notification from RX task, so first audio data is half-valid
    // This should limit delay to just the first swap, i.e. 50 ms
    (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

void print_task_stats(void* pvParameters)
{
    const char* TAG = "Task Stats";
    float* rx_FFT_mag_cpy = (float *)malloc(PLOT_LEN * sizeof(float));
    float* tx_FFT_mag_cpy = (float *)malloc(PLOT_LEN * sizeof(float));

    configASSERT( (rx_FFT_mag_cpy != NULL) && (tx_FFT_mag_cpy != NULL) );

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
        
        ESP_LOGI(TAG, "Running average DSP calc time: %.4f ms, avg num peaks: %.2f, phase comp reset %0d times",
            dsp_calc_time_avg, num_peaks_avg, phase_reset_count);
        ESP_LOGI(TAG, "TX ovfl hit: %0d, RX ovfl hit: %0d, mode switch count: %0d, current mode: %0d",
                 tx_ovfl_hit, rx_ovfl_hit, mode_switch_isr_count, vocoder_mode);

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

        // Get stack watermarks
        UBaseType_t DSPStackWatermark = uxTaskGetStackHighWaterMark(xDSPTaskHandle);
        UBaseType_t RxStackWatermark = uxTaskGetStackHighWaterMark(xRxTaskHandle);
        UBaseType_t TxStackWatermark = uxTaskGetStackHighWaterMark(xTxTaskHandle);
        UBaseType_t ModeSwitchWatermark = uxTaskGetStackHighWaterMark(xModeSwitchTaskHandle);
        UBaseType_t StatsStackWatermark = uxTaskGetStackHighWaterMark(NULL); // This task

        ESP_LOGW(TAG, "Stack Watermarks:");
        ESP_LOGI(TAG, "DSP Task:   %0d", DSPStackWatermark);
        ESP_LOGI(TAG, "Rx Task:    %0d", RxStackWatermark);
        ESP_LOGI(TAG, "Tx Task:    %0d", TxStackWatermark);
        ESP_LOGI(TAG, "Mode Switch Task: %0d", ModeSwitchWatermark);
        ESP_LOGI(TAG, "Stats Task: %0d", StatsStackWatermark);

        size_t free_heap_size = xPortGetFreeHeapSize();
        ESP_LOGW(TAG, "Free heap remaining: %d B", free_heap_size);
    }
}

void mon_mode_switch(void* pvParameters)
{
    int gpio_level;

    // Initialize GPIO interrupt on mode switch button
    init_gpio_button(GPIO_MODE_SWITCH, xModeSwitchTaskHandle);
#ifndef INMP441_MIC
    // Initialize LED output for mode switch
    gpio_set_direction(GPIO_LED_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_LED_2, GPIO_MODE_OUTPUT);
    // Set LEDs to initial mode
    set_mode_leds();
#endif

    while (1)
    {
        // Enable ISR
        gpio_intr_enable(GPIO_MODE_SWITCH);

        // Wait for task notification from ISR
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Disable ISR
        gpio_intr_disable(GPIO_MODE_SWITCH);

        // Perform debounce check on ISR
        if (!gpio_debounce_check(GPIO_MODE_SWITCH, &gpio_level)) {
            // Debounce check failed, just a spurious interrupt
            // Move on and disregard level
            continue;
        }

        if (gpio_level == 0) {
            mode_switch_isr_count++;
            // Button is pressed, get mode switch semaphore
            xSemaphoreTake(xModeSwitchMutex, portMAX_DELAY);
            // Get int value
            int mode_int = (int)vocoder_mode;
            // Increment mode int, with modulo for max number of modes
            mode_int++;
            mode_int %= (int)MAX_VOCODER_MODE;
            // Set enum to value
            vocoder_mode = (vocoder_mode_e)mode_int;
            // Set LEDs to match new vocoder mode, if not in INMP441 mode
        #ifndef INMP441_MIC
            set_mode_leds();
        #endif
            // Give back semaphore
            xSemaphoreGive(xModeSwitchMutex);
        }
        // If 1, do nothing for now

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
    memset(prev_rx_phase, 0.0, sizeof(prev_rx_phase));
    memset(txBuffer_overlap, 0, sizeof(txBuffer_overlap));
    memset(rxBuffer_overlap, 0, sizeof(rxBuffer_overlap));

    // Initialize FFT coefficients
    dsps_fft2r_init_fc32(NULL, N);

    // Set peak shift algorithm config
    peak_shift_cfg_t cfg = {
        .hop_size    = HOP_SIZE,
        .bin_freq_step = 1.0 * I2S_SAMPLING_FREQ_HZ / N_SAMPLES,
        .fft_ptr = rx_FFT,
        .fft_mag_ptr = rx_FFT_mag,
        .fft_phase_ptr = rx_FFT_phase,
        .fft_out_ptr = tx_iFFT,
        .fft_prev_phase = prev_rx_phase,
    };
    init_peak_shift_cfg(&cfg);

    // Reset phase compensation buffer
    reset_phase_comp_arr(run_phase_comp);

    // Intantiate indices
    i2s_idx = I2S_IDX_START;
    dsp_idx = DSP_IDX_START;

    // Create mutex for debug buffers
    xDbgMutex = xSemaphoreCreateMutex();
    // Create semaphore for mode switching
    xModeSwitchMutex = xSemaphoreCreateMutex();

    // Instantiate event group for task sync
    xTaskSyncBits = xEventGroupCreate();

    // Instantiate I2S callbacks
    i2s_event_callbacks_t cbs = {
        .on_recv = rx_rcvd_callback,
        .on_recv_q_ovf = rx_rcvd_overflow,
        .on_sent = NULL, // Not necessary
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

    ESP_LOGW(TAG, "Starting mode switch GPIO task...");
    xReturned |= xTaskCreatePinnedToCore(
        mon_mode_switch, 
        "Mode Switch Task",
        MODE_SWITCH_TASK_STACK_SIZE,
        NULL,
        MODE_SWITCH_TASK_PRIORITY,
        &xModeSwitchTaskHandle,
        MODE_SWITCH_TASK_CORE
    );

    // Verify all tasks started properly
    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create tasks! Error: %d. Restarting ESP in 5 seconds...", xReturned);
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
