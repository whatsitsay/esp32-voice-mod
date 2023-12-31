
/*
 * FFT-based implementation of Schroeder reverb 
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

// Local libraries
#include <filters.h>
#include <es8388.h>
#include <algo_common.h>

// Number of samples
#define HOP_SIZE (N_SAMPLES / 2) // Overlap 50%
#define HOP_BUFFER_SIZE_B (2 * HOP_SIZE * 4) // == length of rx/tx buffers (bytes)
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// I2S macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs

// FFT buffers
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Comb filter parameters
#define NUM_COMB_FILTERS            (4)
#define NUM_AP_FILTERS              (2)
#define COMB_RVT_MS                 (1000.0) // "Reverb time desired", time it takes delayed signal to reach -60 dB
#define AP1_RVT_MS                  (96.83)
#define AP2_RVT_MS                  (32.92)
#define COMB1_TAU_MS                (29.7) // Loop time of comb filter 1
#define COMB2_TAU_MS                (37.1) // Loop time of comb filter 2
#define COMB3_TAU_MS                (41.1) // Loop time of comb filter 2
#define COMB4_TAU_MS                (43.7) // Loop time of comb filter 2
#define AP1_TAU_MS                  (5.0)  // Loop time of allpass filter 1
#define AP2_TAU_MS                  (1.7)  // Loop time of allpass filter 2
#define FILTER_GAIN(tau_ms, rvt_ms) (powf(0.001, tau_ms / rvt_ms)) // Gain based on loop time
#define FFT_MOD_SIZE                ((N_SAMPLES / 2) + 1)

// Array of impulse frequency response coefficients
__attribute__((aligned(16))) float reverb_coeffs[2 * FFT_MOD_SIZE];

// Stats trackers
static unsigned int loop_count  = 0;
static float dsp_calc_time_sum  = 0;
static unsigned int rx_ovfl_hit = 0;
static unsigned int tx_ovfl_hit = 0;

// Debug
#define PLOT_LEN (128)
__attribute__((aligned(16))) float rx_FFT_mag[PLOT_LEN];
__attribute__((aligned(16))) float tx_FFT_mag[PLOT_LEN]; // For debug only

// Ping-pong buffers
#define NUM_BUFFERS (2)
int* txBuffers[NUM_BUFFERS];
int* rxBuffers[NUM_BUFFERS];
int i2s_idx, dsp_idx; // I2S idx should be the same between TX/RX, but opposite from DSP
#define I2S_IDX_START (0)
#define DSP_IDX_START (1)

// Overlap buffers
float txBuffer_overlap[HOP_SIZE];
float rxBuffer_overlap[HOP_SIZE];

// Stream handles
i2s_chan_handle_t rx_handle, tx_handle;

// Task handles
static TaskHandle_t xDSPTaskHandle, xRxTaskHandle, xTxTaskHandle, xTaskStatsHandle;
#define DSP_TASK_STACK_SIZE (16384u) // Check watermark!
#define DSP_TASK_CORE (0)
#define RX_TASK_STACK_SIZE (4096u) // Check watermark!
#define RX_TASK_CORE (1)
#define TX_TASK_STACK_SIZE (4096u) // Check watermark!
#define TX_TASK_CORE (1)
#define TASK_STATS_STACK_SIZE (4096u) // Check watermark!
#define TASK_STATS_CORE (0)

#define DSP_TASK_PRIORITY (1U) // Just above idle
#define TX_TASK_PRIORITY (10U) // Higher due to it being blocked
#define RX_TASK_PRIORITY (10U) // Higher still since it will spend the most time blocked
#define TASK_STATS_PRIORITY (0U) // == IDLE PRIO

// Event group
EventGroupHandle_t xTaskSyncBits;
#define DSP_TASK_BIT      ( 1 << 0 )
#define TX_TASK_BIT       ( 1 << 1 )
#define RX_TASK_BIT       ( 1 << 2 )
#define SWAP_COMPLETE_BIT ( 1 << 3 )
#define BUFF_SWAP_BITS (DSP_TASK_BIT | TX_TASK_BIT | RX_TASK_BIT) // Sync to initiate buffer swap
#define ALL_SYNC_BITS  (BUFF_SWAP_BITS | SWAP_COMPLETE_BIT) // Sync to move on
#define SYNC_TIMEOUT_TICKS  (500 / portTICK_PERIOD_MS) // Raise error if not synced by this point

// Semaphores
static SemaphoreHandle_t xDbgMutex;

////// HELPER FUNCTIONS //////

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
        
        ESP_LOGI(TAG, "Running average DSP calc time: %.4f ms", dsp_calc_time_avg);
        ESP_LOGI(TAG, "TX overflow hit count: %0d, RX overflow hit count: %0d", tx_ovfl_hit, rx_ovfl_hit);

        // Clear sum and count
        dsp_calc_time_sum = 0;
        loop_count = 0;

        // Copy magnitude buffers
        memcpy(rx_FFT_mag_cpy, rx_FFT_mag, PLOT_LEN * sizeof(float));
        memcpy(tx_FFT_mag_cpy, tx_FFT_mag, PLOT_LEN * sizeof(float));

        // Release mutex
        xSemaphoreGive(xDbgMutex);

        // Plot magnitudes
        ESP_LOGI(TAG, "Input FT magnitude (dB):");
        dsps_view(rx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, 0, 40, 'x');
        ESP_LOGI(TAG, "Output FT magnitude (dB):");
        dsps_view(tx_FFT_mag_cpy, PLOT_LEN, PLOT_LEN, 10, 0, 40, 'o');
    }
}

void init_reverb_coeffs()
{
    // Calculate all gains and store in temporary luts
    float comb_gain[] = {
        FILTER_GAIN(COMB1_TAU_MS, COMB_RVT_MS),
        FILTER_GAIN(COMB2_TAU_MS, COMB_RVT_MS),
        FILTER_GAIN(COMB3_TAU_MS, COMB_RVT_MS),
        FILTER_GAIN(COMB4_TAU_MS, COMB_RVT_MS)
    };
    float ap_gain[] = {
        FILTER_GAIN(AP1_TAU_MS, AP1_RVT_MS),
        FILTER_GAIN(AP2_TAU_MS, AP2_RVT_MS)
    };


    // Calculate all delay indicies, which will be delay (loop) time x sampling frequency
    int comb_delay_idx[] = {
        roundf(COMB1_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000),
        roundf(COMB2_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000),
        roundf(COMB3_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000),
        roundf(COMB4_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000)
    };
    int ap_delay_idx[] = {
        roundf(AP1_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000),
        roundf(AP2_TAU_MS * I2S_SAMPLING_FREQ_HZ / 1000)
    };

    // Loop over coefficient LUT
    for (int k = 0; k < FFT_MOD_SIZE; k++)
    {
        // Start with impulse 1 + 0j
        float ap_real = 1; 
        float ap_imag = 0;
        // Cascade multiply allpass coefficients
        for (int i = 0; i < NUM_AP_FILTERS; i++) {
            allpass_filter(ap_gain[i], ap_delay_idx[i] * k, ap_real, ap_imag, &ap_real, &ap_imag);
        }

        // Calculate comb filters in parallel, so each gets an impulse response as input
        float comb_real = 0;
        float comb_imag = 0;
        for (int i = 0; i < NUM_COMB_FILTERS; i++) {
            float curr_comb_real, curr_comb_imag;
            comb_filter(comb_gain[i], comb_delay_idx[i] * k, 1, 0, &curr_comb_real, &curr_comb_imag);
            // Comb filters are summed, so add to running sum here
            // Also correct for number of filters when adding
            comb_real += curr_comb_real / NUM_COMB_FILTERS;
            comb_imag += curr_comb_imag / NUM_COMB_FILTERS;
        }

        // Reverb coefficients will be product of AP and comb filters
        mult_complex(ap_real, ap_imag, comb_real, comb_imag, &reverb_coeffs[2*k], &reverb_coeffs[2*k + 1]);
    }
}

/**
 * Function for performing audio data modification
 * 
 * In this app, performs FFT and iFFT on data, essentially acting
 * as a passthrough
 */
void audio_data_modification(int* txBuffer, int* rxBuffer) {
    static const char* TAG = "Audio Modification";

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
    // Calculate magnitudes, for debug
    calc_fft_mag_db(rx_FFT, rx_FFT_mag, PLOT_LEN);

    // Multiply each value by reverb coefficient, calculated prior
    for (int i = 0; i < 2 * FFT_MOD_SIZE; i += 2) {
        float reverb_real = reverb_coeffs[i];
        float reverb_imag = reverb_coeffs[i+1];
        float in_real     = rx_FFT[i];
        float in_imag     = rx_FFT[i+1];
        float *out_real   = &tx_iFFT[i];
        float *out_imag   = &tx_iFFT[i+1];

        mult_complex(reverb_real, reverb_imag, in_real, in_imag, out_real, out_imag);
    }
    // REMOVE ME: just copy rx_FFT->tx_iFFT
    // memcpy(tx_iFFT, rx_FFT, FFT_MOD_SIZE * 2 * sizeof(float));

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
        audio_data_modification(txBuffer, rxBuffer);
        
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

    // Initialize complex sinusoid coefficients
    

    // Initialize reverb coefficients
    init_reverb_coeffs();

    // Intantiate indices
    i2s_idx = I2S_IDX_START;
    dsp_idx = DSP_IDX_START;

    // Create mutex for debug buffers
    xDbgMutex = xSemaphoreCreateMutex();

    // Instantiate event group for task sync
    xTaskSyncBits = xEventGroupCreate();

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
