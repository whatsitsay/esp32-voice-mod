
/*
 * Microphone test for INMP441 breakout board
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
#include <es8388.h>

// Number of samples
#define N_SAMPLES (4096)
#define HOP_SIZE (N_SAMPLES / 2) // Overlap 50%
#define HOP_BUFFER_SIZE_B (2 * HOP_SIZE * 4) // == length of rx/tx buffers (bytes)
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// I2S macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs

// FFT buffers
__attribute__((aligned(16))) float hann_win[N_SAMPLES];
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Stats trackers
static unsigned int loop_count = 0;
static float fft_calc_time_sum = 0;
static unsigned int rx_ovfl_hit = 0;
static unsigned int tx_ovfl_hit = 0;

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

// Debug buffers
float rx_dbg[HOP_SIZE];
float tx_dbg[HOP_SIZE];

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
#define TASK_STATS_STACK_SIZE (4096u)
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

    // Correct all values by sample size and window
    // Correct imaginary components (which should be close to 0) to conjugate
    for (int i = 0; i < num_samples; i++)
    {
        fft_arr[2 * i] /= (float)num_samples; // Correction factor
        // fft_arr[2 * i] *= hann_win[i]; // For proper reconstruction
        // Conjugate of imaginary component (should be close to 0)
        fft_arr[2 * i + 1] /= (float)num_samples;
        fft_arr[2 * i + 1] *= -1; // Complex conjugate
        // fft_arr[2 * i + 1] *= hann_win[i]; // Proper reconstruction (less necessary)
    }

    return ESP_OK;
}

void print_task_stats(void* pvParameters)
{
    const char* TAG = "Task Stats";

    while (1) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Grab mutex
        if (xSemaphoreTake(xDbgMutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to get debug buffer mutex for stats");
            return;
        }

        // Calculate average time spent calculating FFT/iFFT in ms
        float fft_calc_time_avg = fft_calc_time_sum / loop_count;
        // Calculate average error per element of most recent sample
        float avg_element_err_pct = 0;
        for (int i = 0; i < HOP_SIZE; i++)
        {
            float rx_tx_diff_raw = tx_dbg[i] - rx_dbg[i];
            float rx_tx_diff_pct = (rx_dbg[i] != 0) ? rx_tx_diff_raw / fabsf(rx_dbg[i]) :
                                (tx_dbg[i] == 0) ? 0 : 1.0;
            rx_tx_diff_pct *= 100.0;
            avg_element_err_pct += fabsf(rx_tx_diff_pct);
        }
        avg_element_err_pct /= HOP_SIZE;
        
        ESP_LOGI(TAG, "Running average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
            fft_calc_time_avg, avg_element_err_pct);
        ESP_LOGI(TAG, "TX overflow hit count: %0d, RX overflow hit count: %0d", tx_ovfl_hit, rx_ovfl_hit);

        // Clear sum and count
        fft_calc_time_sum = 0;
        loop_count = 0;

        // Release mutex
        xSemaphoreGive(xDbgMutex);
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
    memcpy(rx_dbg, rxBuffer_overlap, sizeof(rx_dbg));

    unsigned int start_cc = dsp_get_cpu_cycle_count();

    // Copy values of rxBuffer into FFT buffer
    for (int i = 0; i < N_SAMPLES; i++) {
        // Select sample. If first half, use rxBuffer_overlap. Otherwise use rxBuffer
        int rx_val = (i < HOP_SIZE) ? rxBuffer_overlap[i] : rxBuffer[2 * (i - HOP_SIZE)];

        // Dot-product with hann window
        // Downshift to prevent overflow (last 8 bits are always 0)
        rx_FFT[2 * i] = (float)(rx_val >> I2S_DOWNSHIFT) * hann_win[i];
        // Set imaginary component to 0
        rx_FFT[2 * i + 1] = 0;
    }

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

    // Perform iFFT calc
    inv_fft(tx_iFFT, N);
    
    unsigned int end_cc = dsp_get_cpu_cycle_count();

    // Calculate time spent, add to running sum
    fft_calc_time_sum += (float)(end_cc - start_cc) / 240e3;
    loop_count++;

    // Fill TX buffer
    for (int i = 0; i < HOP_SIZE; i++)
    {
        // Add-overlay beginning portion of iFFT into txBuffer
        float tx_val = tx_iFFT[2 * i];
        txBuffer[2 * i] = (int)(txBuffer_overlap[i] + tx_val); 
        txBuffer[2 * i] <<= I2S_DOWNSHIFT; // Increase int value
        txBuffer[2 * i + 1] = txBuffer[2 * i]; // Copy L and R

        // Store TX val to debug array for later comparison
        tx_dbg[i] = (txBuffer_overlap[i] + tx_val) * 256;

        // Store latter portion for use next loop
        float tx_overlap_val = tx_iFFT[2 * (i + HOP_SIZE)];
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

        // Modify data
        audio_data_modification(txBuffer, rxBuffer);

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

    // Intantiate indices
    i2s_idx = I2S_IDX_START;
    dsp_idx = DSP_IDX_START;

    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

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

    ESP_LOGW(TAG, "Starting RX task..."); // Last to only start popping data when ready
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
        "I2S Transmit Task",
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
