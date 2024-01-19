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
#include <filters.h>
#include "esp_vocoder_main.h"
#include <Yin.h>

////// HELPER FUNCTIONS //////

/**
 * @brief Function for performing audio data modification
 * 
 * @param rxBuffer - Buffer containing microphone input sound data
 * @param txBuffer - Output buffer for modified sound data for transmission
 */
void audio_data_modification() {
    static vocoder_mode_e prev_vocoder_mode = MOD_CHORUS;
    static unsigned silence_count = 0;

    // Copy in previous input FFT
    memcpy(prev_rx_FFT, rx_FFT, sizeof(prev_rx_FFT));

    // Copy values of rxBuffer into FFT buffer
    for (int i = 0; i < N_SAMPLES; i++) {
        // Get sample
        int rx_val = dsp_rx[i];
        // Dot-product with hann window
        // Downshift to prevent overflow (last 8 bits are always 0)
        rx_FFT[2 * i] = (float)(rx_val / I2S_DOWNSHIFT_DIV) * get_window(i);
        // Set imaginary component to 0
        rx_FFT[2 * i + 1] = 0;
    }

    // FFT Calculation
    ESP_ERROR_CHECK(calc_fft(rx_FFT, N));

    // Magnitude calculation
    float max_mag_db = calc_fft_mag_db(rx_FFT, rx_FFT_mag, FFT_MOD_SIZE);
    // Change to raw log for true env calculation
    dsps_mulc_f32(rx_FFT_mag, rx_FFT_mag, FFT_MOD_SIZE, 0.05, 1, 1);

    // Find peaks
    int num_peaks = find_local_peaks();

    num_peaks_sum += num_peaks;

    // Calculate true envelope, using fundamental frequency estimate from peak finding
    calc_true_envelope(rx_FFT_mag, rx_env, fundamental_freq_est + CEPSTRUM_BUFF);
    // Convert to raw from log for pre-warping
    for (int i = 0; i < FFT_MOD_SIZE; i++) {
        rx_env[i] = pow10f(rx_env[i]/10);
        // Calc inverse
        rx_env_inv[i] = 1 / rx_env[i];
    }
    // Return to using dB in mag plot
    dsps_mulc_f32(rx_FFT_mag, rx_FFT_mag, FFT_MOD_SIZE, 20, 1, 1);

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
        if (silence_count % SILENCE_RESET_COUNT == 0) {
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
            // Copy in original sound (save cycles)
            memcpy(tx_iFFT, rx_FFT, FFT_MOD_SIZE * 2 * sizeof(float));
            // Perform full chorus shift for the rest
            for (int i = 0; i < NUM_PITCH_SHIFTS; i++) {
                shift_peaks(PITCH_SHIFT_FACTORS[i], PITCH_SHIFT_GAINS[i], fundamental_freq_idx, run_phase_comp);
            }
            break;
        }
        case MOD_LOW: {
            shift_peaks(LOW_EFFECT_SHIFT, LOW_EFFECT_GAIN, fundamental_freq_idx, run_phase_comp);
            break;
        }
        case MOD_HIGH: {
            shift_peaks(HIGH_EFFECT_SHIFT, HIGH_EFFECT_GAIN, fundamental_freq_idx, run_phase_comp);
            break;
        }
        case PASSTHROUGH: {
            // NOTE: this could just be another copy situation
            shift_peaks(PASSTHROUGH_SHIFT, PASSTHROUGH_GAIN, fundamental_freq_idx, run_phase_comp);
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

    // Fill latter half of FFT with conjugate mirror data
    fill_mirror_fft(tx_iFFT, N);

    // Perform iFFT calc
    ESP_ERROR_CHECK(inv_fft(tx_iFFT, N));

    // Fill TX buffer
    // First half will be overlap-add of previous data and current
    for (int i = 0; i < HOP_SIZE; i++)
    {
        float tx_new     = tx_iFFT[2 * i] * get_window(i); // Window as former half
        float tx_overlap = dsp_tx[i] * get_window(i + HOP_SIZE); // Window as latter half
        // Sum for overlap-add
        dsp_tx[i] = (int)roundf(tx_new + tx_overlap);
        // Correct for bitshift
        dsp_tx[i] <<= I2S_DOWNSHIFT;
    }
    // Second half will be overlap data
    for (int i = HOP_SIZE; i < N_SAMPLES; i++)
    {
        dsp_tx[i] = (int)roundf(tx_iFFT[2 * i]); // Store as rounded value
    }
}

void set_mode_leds()
{
    int mode_int = (int)vocoder_mode;
    // Set to the opposite mode of each bit, as they need to be pulled down
    gpio_set_level(GPIO_LED_1, (~mode_int) & 0x1);
    gpio_set_level(GPIO_LED_2, ~(mode_int >> 1) & 0x1);
}

///// CALLBACKS ///// 

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

    // Register with WDT
    esp_task_wdt_add(NULL);

    // Main loop
    while (1) {
        // Wait for RX bit to be set
        (void) xEventGroupWaitBits(RxSync, I2S_SYNC_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // Move up delay tap
        // FIXME need to work in delay tap, if desired
        memcpy(dsp_rx, dsp_rx + HOP_SIZE, N_SAMPLES * sizeof(int));
        // Set individual bit and move on
        (void) xEventGroupSetBits(RxSync, DSP_SYNC_BIT);

        unsigned int start_cc = dsp_get_cpu_cycle_count();

        // Modify data
        audio_data_modification();
        
        unsigned int end_cc = dsp_get_cpu_cycle_count();

        // Calculate time spent, add to running sum
        volatile float iter_calc_time = (float)(end_cc - start_cc) / 240e3;
        // Check against buffering time for I2S, with a little extra in case
        // If this is failed, then the audio will be choppy
        if (iter_calc_time > 1.5 * I2S_BUFFER_TIME_MS) {
            ESP_LOGE(TAG, "DSP calculation time too long! Should be <= %.1f ms, is instead %.3f", I2S_BUFFER_TIME_MS, iter_calc_time);
            configASSERT(false);
        }
        else if (iter_calc_time > I2S_BUFFER_TIME_MS) {
            // Just issue warning
            ESP_LOGW(TAG, "DSP calculation time a little too long at %.3f ms", iter_calc_time);
        }

        dsp_calc_time_sum += iter_calc_time;
        loop_count++;

        // Reset watchdog
        esp_task_wdt_reset();

        // Move up buffer for I2S
        memcpy(i2s_tx, dsp_tx, N_SAMPLES * sizeof(int));

        // Set event group bit and wait for sync
        (void) xEventGroupSync(TxSync,
                               DSP_SYNC_BIT,
                               ALL_SYNC_BITS,
                               portMAX_DELAY);
        // Clear both bits
        (void) xEventGroupClearBits(TxSync, ALL_SYNC_BITS);
    }
    
}

void i2s_receive(void* pvParameters)
{
    static const char *TAG = "I2S receive";

    size_t data_received = 0;

    configASSERT( rx_handle != NULL );
    
    // Instantiate stream buffer
    int* rx_stream_buff = (int *)pvParameters;
    configASSERT( rx_stream_buff != NULL );

    // Enable RX channel
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    while (1) {
        // Wait for sent notification
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int* i2s_rx_ptr = i2s_rx;

        // Read from I2S buffer
        for (int i = 0; i < RX_STREAM_BUFF_DIV; i++) {
            i2s_channel_read(rx_handle, rx_stream_buff, RX_STREAM_BUFF_SIZE_B, &data_received, 300 / portTICK_PERIOD_MS);
            if (data_received != RX_STREAM_BUFF_SIZE_B) {
                ESP_LOGE(TAG, "Only read %d out of %d bytes from I2S buffer", data_received, RX_STREAM_BUFF_SIZE_B);
            }

            // Fill I2S RX buffer
            // For now just use even-indexed values for left mic
            for (int j = 0; j < HOP_SIZE / RX_STREAM_BUFF_DIV; j++) {
                *i2s_rx_ptr = rx_stream_buff[2 * j];
                i2s_rx_ptr++; // Increment pointer
            }
        }

        // Calculate fundamental freq estimate using Yin algorithm
        float hop_freq = Yin_getPitch(&yin_s, i2s_rx, I2S_SAMPLING_FREQ_HZ);
        // Only latch as fundamental if value isn't -1
        if (hop_freq > -1) {
            // Use LPF
            fundamental_freq_est = (1 - F0_EST_FB_FACTOR) * hop_freq + F0_EST_FB_FACTOR;
            // Round to get index; used for differentiating between above/below F0
            fundamental_freq_idx = roundf((fundamental_freq_est / I2S_SAMPLING_FREQ_HZ) * N_SAMPLES);
        }

        // Set event group bit, wait for all sync before moving on
        (void) xEventGroupSync(RxSync,
                               I2S_SYNC_BIT,
                               ALL_SYNC_BITS,
                               portMAX_DELAY);

        // Clear all bits
        (void) xEventGroupClearBits(RxSync, ALL_SYNC_BITS);
    }
}

void i2s_transmit(void* pvParameters)
{
    static const char *TAG = "I2S transmit";

    configASSERT( tx_handle != NULL );

    // Instantiate stream buffer
    int* tx_stream_buff = (int *)pvParameters;
    configASSERT( tx_stream_buff != NULL );
    
    // Enable TX channel
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    while (1) {
        size_t data_written = 0;

        // Wait for DSP sync bit to be set before continuing on
        (void) xEventGroupWaitBits(TxSync, DSP_SYNC_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

        // Copy in DSP data to stream buffer
        for (int j = 0; j < HOP_SIZE; j++) {
            tx_stream_buff[2 * j]     = i2s_tx[j];
            tx_stream_buff[2 * j + 1] = i2s_tx[j];
        }

        // Set bit and release DSP task
        (void) xEventGroupSetBits(TxSync, I2S_SYNC_BIT);

        // Write to I2S buffer
        // No error check here to allow for partial writes
        i2s_channel_write(tx_handle, tx_stream_buff, HOP_BUFFER_SIZE_B, &data_written, 300 / portTICK_PERIOD_MS);
        if (data_written != HOP_BUFFER_SIZE_B) {
            ESP_LOGE(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, HOP_BUFFER_SIZE_B);
        }
    }
}

void print_task_stats(void* pvParameters)
{
    const char* TAG = "Task Stats";

    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);

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

        // Print local peaks
        print_local_peaks();
        // Print fundamental frequency estimate
        ESP_LOGI(TAG, "Fundamental frequency est: %.2f Hz (prob %.4f)", fundamental_freq_est, yin_s.probability);

        // Copy magnitude buffers
        // memcpy(rx_FFT_mag_cpy, rx_FFT_mag, PLOT_LEN * sizeof(float));
        // memcpy(tx_FFT_mag_cpy, tx_FFT_mag, PLOT_LEN * sizeof(float));

        // Plot magnitudes
        // ESP_LOGI(TAG, "Input FT magnitude (dB):");
        // dsps_view(rx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, -30, 10, 'x');
        // ESP_LOGI(TAG, "Output FT magnitude (dB):");
        // dsps_view(tx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, -30, 10, 'o');

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
        ESP_LOGW(TAG, "Free heap remaining: %d B\n", free_heap_size);
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
    static bool init_speaker_silence = true;
    BaseType_t xReturned = 0UL;
    
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);
    // Start silent
    es8388_set_voice_mute(init_speaker_silence);

    ESP_LOGW(TAG, "Channels initiated!");

    // Allocate TX/RX data buffers
    rxBuffer = calloc(FULL_BUFFER_SIZE, sizeof(int));
    txBuffer = calloc(FULL_BUFFER_SIZE, sizeof(int));
    configASSERT(rxBuffer != NULL && txBuffer != NULL);

    // Allocate stream buffers
    void* rx_stream_buff = calloc(2 * HOP_SIZE / RX_STREAM_BUFF_DIV, sizeof(int)); // Smaller to save on memory footprint
    void* tx_stream_buff = calloc(2 * HOP_SIZE, sizeof(int));

    // Clear out all other memory buffers
    memset(rx_FFT, 0, sizeof(rx_FFT));
    memset(prev_rx_FFT, 0, sizeof(prev_rx_FFT));
    memset(tx_iFFT, 0.0, sizeof(tx_iFFT));

    // Initialize FFT coefficients
    dsps_fft2r_init_fc32(NULL, N);

    // Initialize Yin algorithm struct
    Yin_init(&yin_s, YIN_SAMPLES, YIN_DEFAULT_THRESHOLD, yinBuffPtr);

    // Set peak shift algorithm config
    peak_shift_cfg_t cfg = {
        .hop_size    = HOP_SIZE,
        .bin_freq_step = 1.0 * I2S_SAMPLING_FREQ_HZ / N_SAMPLES,
        .fft_ptr = rx_FFT,
        .fft_prev_ptr = prev_rx_FFT,
        .fft_mag_ptr = rx_FFT_mag,
        .fft_out_ptr = tx_iFFT,
        .true_env_ptr = rx_env,
        .inv_env_ptr  = rx_env_inv,
    };
    init_peak_shift_cfg(&cfg);

    // Reset phase compensation buffer
    reset_phase_comp_arr(run_phase_comp);

    // Config true envelope calculation
    config_true_env_calc(env_FFT, cepstrum_buff, I2S_SAMPLING_FREQ_HZ);

    // Create semaphore for mode switching
    xModeSwitchMutex = xSemaphoreCreateMutex();

    // Instantiate sync event groups
    RxSync = xEventGroupCreate();
    TxSync = xEventGroupCreate();

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
    // Pass in pre-allocated stream buffers
    ESP_LOGW(TAG, "Starting RX task...");
    xReturned |= xTaskCreatePinnedToCore(
        i2s_receive, 
        "I2S Receive Task",
        RX_TASK_STACK_SIZE,
        rx_stream_buff,
        RX_TASK_PRIORITY,
        &xRxTaskHandle,
        RX_TASK_CORE
    );
    ESP_LOGW(TAG, "Starting TX task...");
    xReturned |= xTaskCreatePinnedToCore(
        i2s_transmit, 
        "I2S Transmit Task",
        TX_TASK_STACK_SIZE,
        tx_stream_buff,
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
        ESP_LOGE(TAG, "Failed to create tasks! Error: %d", xReturned);
        configASSERT(0);
    }

    // Loop forever
    while(1) {
        vTaskDelay(500 / portTICK_PERIOD_MS);

        taskENTER_CRITICAL(&crit_mux);
        // Toggle headphone jack
        es_toggle_power_amp();

        taskEXIT_CRITICAL(&crit_mux);

        // If silent, unmute
        if (init_speaker_silence) {
            es8388_set_voice_mute(false);
            init_speaker_silence = false;
        }
    };
}
