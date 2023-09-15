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
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_dsp.h"

// Local libraries
#include <es8388.h>

// Allocate buffer
#define SAMPLES_PER_AVG (100)
#define BYTES_PER_SAMPLE (4) // According to spec, 32bits/word
#define N_SAMPLES (4096)
#define RX_TX_BUFFER_LEN (N_SAMPLES / 2) // Try bigger spec
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// Plot macros
#define PLOT_LEN (RX_TX_BUFFER_LEN / 8)
#define PLOT_MAX (powf(2, 24))

// I2S macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Lower for more even freq resolution
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs

// FFT buffers
__attribute__((aligned(16))) float hann_win[N_SAMPLES];
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];

// Instantiate pointers to debug buffers
float rx_dbg[RX_TX_BUFFER_LEN];
float tx_dbg[RX_TX_BUFFER_LEN];

// Avg sum values
static unsigned int loop_count = 0;
static float fft_calc_time_sum = 0;

// Instantiate pointers to RX/TX buffers 
static int txBuffer[RX_TX_BUFFER_LEN * 2]; // L + R
static float txBuffer_overlap[RX_TX_BUFFER_LEN];
static int rxBuffer[RX_TX_BUFFER_LEN * 2]; // L + R
static float rxBuffer_overlap[RX_TX_BUFFER_LEN];

// Stream handles
i2s_chan_handle_t rx_handle, tx_handle;
StreamBufferHandle_t xRxStreamBuffer, xTxStreamBuffer;
#define STREAM_BUFFER_SIZE_B (3 * RX_TX_BUFFER_LEN * 4) // Slightly bigger for some overflow
#define RX_READ_SIZE_B (2 * RX_TX_BUFFER_LEN * 4) // == length of rx/tx buffers
#define TX_POP_SIZE (RX_TX_BUFFER_LEN / 2) // Smaller to allow tasks to trade off
#define TX_POP_SIZE_B (TX_POP_SIZE * 4)

// Task handles
static TaskHandle_t xDSPTaskHandle, xRxTaskHandle, xTxTaskHandle;
#define DSP_TASK_STACK_SIZE (16384u) // Check watermark!
#define DSP_TASK_CORE (0)
#define RX_TASK_STACK_SIZE (4096u) // Check watermark!
#define RX_TASK_CORE (1)
#define TX_TASK_STACK_SIZE (4096u) // Check watermark!
#define TX_TASK_CORE (1)

static SemaphoreHandle_t xRxBufferMutex, xTxBufferMutex;

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

void print_task_stats()
{
    const char* TAG = "Task Stats";

    // Grab mutex
    if (xSemaphoreTake(xRxBufferMutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get RX buffer mutex for stats");
        return;
    }
    if (xSemaphoreTake(xTxBufferMutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get TX buffer mutex for stats");
        return;
    }

    // Calculate average time spent calculating FFT/iFFT in ms
    float fft_calc_time_avg = fft_calc_time_sum / loop_count;
    // Calculate average error per element of most recent sample
    float avg_element_err_pct = 0;
    for (int i = 0; i < RX_TX_BUFFER_LEN; i++)
    {
        float rx_tx_diff_raw = tx_dbg[i] - rx_dbg[i];
        float rx_tx_diff_pct = (rx_dbg[i] != 0) ? rx_tx_diff_raw / fabsf(rx_dbg[i]) :
                            (tx_dbg[i] == 0) ? 0 : 1.0;
        rx_tx_diff_pct *= 100.0;
        avg_element_err_pct += fabsf(rx_tx_diff_pct);
    }
    avg_element_err_pct /= RX_TX_BUFFER_LEN;
    
    ESP_LOGI(TAG, "Running average FFT/iFFT calc time: %.4f ms, avg error: %.2f %%",
        fft_calc_time_avg, avg_element_err_pct);

    // Clear sum and count
    fft_calc_time_sum = 0;
    loop_count = 0;

    // Release mutexes
    xSemaphoreGive(xRxBufferMutex);
    xSemaphoreGive(xTxBufferMutex);
}

/**
 * Function for performing audio data modification
 * 
 * In this app, performs FFT and iFFT on data, essentially acting
 * as a passthrough
 */
void audio_data_modification() {
    static const char* TAG = "Audio Modification";

    // Copy RX overlap into debug buffer
    if (xSemaphoreTake(xRxBufferMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get mutex for RX debug buffer!");
    }
    memcpy(rx_dbg, rxBuffer_overlap, sizeof(rx_dbg));
    xSemaphoreGive(xRxBufferMutex);

    unsigned int start_cc = dsp_get_cpu_cycle_count();

    // Copy values of rxBuffer into FFT buffer
    for (int i = 0; i < N_SAMPLES; i++) {
        // Select sample. If first half, use rxBuffer_overlap. Otherwise use rxBuffer
        int rx_val = (i < RX_TX_BUFFER_LEN) ? rxBuffer_overlap[i] : rxBuffer[2 * (i - RX_TX_BUFFER_LEN)];

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

    if (xSemaphoreTake(xTxBufferMutex, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to get mutex for TX debug buffer!");
    }
    // Fill TX buffer
    for (int i = 0; i < RX_TX_BUFFER_LEN; i++)
    {
        // Add-overlay beginning portion of iFFT into txBuffer
        float tx_val = tx_iFFT[2 * i];
        txBuffer[2 * i] = (int)(txBuffer_overlap[i] + tx_val); 
        txBuffer[2 * i] <<= I2S_DOWNSHIFT; // Increase int value
        txBuffer[2 * i + 1] = txBuffer[2 * i]; // Copy L and R

        // Store TX val to debug array for later comparison
        tx_dbg[i] = (txBuffer_overlap[i] + tx_val) * 256;

        // Store latter portion for use next loop
        float tx_overlap_val = tx_iFFT[2 * (i + RX_TX_BUFFER_LEN)];
        txBuffer_overlap[i] = tx_overlap_val;
    }
    xSemaphoreGive(xTxBufferMutex);
}

///// CALLBACKS ///// 

// Called whenever buffer has received data. May be unnecessary
static IRAM_ATTR bool rx_rcvd_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Set task notification for RX task to continue
    configASSERT( xRxTaskHandle != NULL );
    vTaskNotifyGiveFromISR(xRxTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    return false; // FIXME what does this boolean indicate? Docs not clear

}

// Called whenever buffer for output has been emptied
static IRAM_ATTR bool tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Set task notification for filler task to continue
    configASSERT( xTxTaskHandle != NULL );
    vTaskNotifyGiveFromISR(xTxTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    return false; // FIXME what does this boolean indicate? Docs not clear
}

///// TASKS ///// 

void proc_audio_data(void* pvParameters)
{
    static const char *TAG = "dsp_loop";

    // Subscribe to watchdog timer (NULL->current task)
    esp_task_wdt_add(NULL);
    
    // Main loop
    while (1) {
        size_t bytes_read = 0, bytes_written = 0;

        // Copy rxBuffer into rxBuffer_overlap
        for (int i = 0; i < RX_TX_BUFFER_LEN; i++)
        {
            rxBuffer_overlap[i] = rxBuffer[2 * i];
        }

        // Pop from receive buffer
        bytes_read = xStreamBufferReceive(xRxStreamBuffer, rxBuffer, sizeof(rxBuffer), 5000 / portTICK_PERIOD_MS);
        if (bytes_read != sizeof(rxBuffer)) {
            ESP_LOGE(TAG, "Failed to read all rxBuffer data from stream buffer! %0d out of %0d B",
                     bytes_read, sizeof(rxBuffer));
        }

        // Modify data
        audio_data_modification();

        // Write to stream buffer
        bytes_written = xStreamBufferSend(xTxStreamBuffer, txBuffer, STREAM_BUFFER_SIZE_B, 10000 / portTICK_PERIOD_MS);
        if (bytes_written != STREAM_BUFFER_SIZE_B) {
            ESP_LOGE(TAG, "Failed to write all txBuffer data to stream buffer! %0d B out of %0d B",
                    bytes_written, STREAM_BUFFER_SIZE_B);
        }

        // Reset watchdog timeout
        ESP_ERROR_CHECK(esp_task_wdt_reset());
    }
    
}

void i2s_receive(void* pvParameters)
{
    static const char *TAG = "I2S receive";
    int* stream_data = (int *)malloc(RX_READ_SIZE_B);

    while (1) {
        size_t data_received, data_pushed = 0;
        // Wait for task notification from callback
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Read from I2S buffer
        ESP_ERROR_CHECK(i2s_channel_read(rx_handle, stream_data, RX_READ_SIZE_B, &data_received, 5000 / portTICK_PERIOD_MS));
        if (data_received != RX_READ_SIZE_B) {
            ESP_LOGE(TAG, "Only read %d out of %d bytes from I2S buffer", data_received, data_pushed);
            if (data_received == 0) continue;
        }

        // Pop data from stream
        data_pushed = xStreamBufferSend(xRxStreamBuffer, stream_data, data_received, portMAX_DELAY);
        if (data_pushed != data_received) {
            ESP_LOGE(TAG, "Only pushed %d out of %d bytes to stream", data_pushed, data_received);
        }
    }
    free(stream_data);
}

void i2s_transmit(void* pvParameters)
{
    size_t data_popped, data_written;
    static const char *TAG = "I2S transmit";
    int* stream_data = (int *)malloc(TX_POP_SIZE_B);

    while (1) {
        // Pop data from stream
        data_popped = xStreamBufferReceive(xTxStreamBuffer, stream_data, TX_POP_SIZE_B, portMAX_DELAY);
        if (data_popped != TX_POP_SIZE_B) {
            ESP_LOGW(TAG, "Only popped %d out of %d bytes from stream", data_popped, TX_POP_SIZE_B);
            if (data_popped == 0) continue;
        }

        // Write to I2S buffer
        ESP_ERROR_CHECK(i2s_channel_write(tx_handle, stream_data, data_popped, &data_written, 1000));
        if (data_written != data_popped) {
            ESP_LOGW(TAG, "Only wrote %d out of %d bytes to I2S buffer", data_written, data_popped);
        }

        // Wait for task notification from callback
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    free(stream_data);
}

///// MAIN LOOP ///// 

void app_main(void)
{
    static const char *TAG = "main";
    BaseType_t xReturned;
    
    // Initiallize ES8388 and I2S channel
    es8388_config();
    es_i2s_init(&tx_handle, &rx_handle, I2S_SAMPLING_FREQ_HZ);

    ESP_LOGW(TAG, "Channels initiated!");
    // Instantiate stream buffers
    xRxStreamBuffer = xStreamBufferCreate(STREAM_BUFFER_SIZE_B, RX_READ_SIZE_B);
    xTxStreamBuffer = xStreamBufferCreate(STREAM_BUFFER_SIZE_B, TX_POP_SIZE_B);
    if (xRxStreamBuffer == NULL || xTxStreamBuffer == NULL) 
    {
        ESP_LOGE(TAG, "Failed to instantiate stream buffers!");
    }
    else ESP_LOGW(TAG, "Stream buffers instantiated!");

    // Generate sin/cos coefficients for FFT calculations
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, N_SAMPLES));

    // Generate Hann window coefficients
    dsps_wind_hann_f32(hann_win, N_SAMPLES);

    // Create mutexes for buffers
    xRxBufferMutex = xSemaphoreCreateMutex();
    xTxBufferMutex = xSemaphoreCreateMutex();

    // Start RX/TX buffer tasks
    ESP_LOGW(TAG, "Starting RX task...");
    xReturned = xTaskCreatePinnedToCore(
        i2s_receive, 
        "I2S Receive Task",
        RX_TASK_STACK_SIZE,
        NULL,
        10, // Can be higher, since it should spend most time blocked
        &xRxTaskHandle,
        RX_TASK_CORE
    );
    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create RX task! Error: %d.", xReturned);
    }
    else
    {
        ESP_LOGW(TAG, "Instantiated RX task successfully!");
    }
    ESP_LOGW(TAG, "Starting TX task...");
    xReturned = xTaskCreatePinnedToCore(
        i2s_transmit, 
        "I2S Transmit Task",
        TX_TASK_STACK_SIZE,
        NULL,
        10, // Can be higher, since it should spend most time blocked
        &xTxTaskHandle,
        TX_TASK_CORE
    );
    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create TX task! Error: %d.", xReturned);
    }
    else
    {
        ESP_LOGW(TAG, "Instantiated RX task successfully!");
    }

    // Instantiate I2S callbacks
    i2s_event_callbacks_t cbs = {
        .on_recv = rx_rcvd_callback,
        .on_recv_q_ovf = NULL,
        .on_sent = tx_sent_callback,
        .on_send_q_ovf = NULL, // May be necessary
    };
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &cbs, NULL));
    ESP_ERROR_CHECK(i2s_channel_register_event_callback(tx_handle, &cbs, NULL));

    // Start DSP task
    ESP_LOGW(TAG, "Starting DSP task...");
    xReturned = xTaskCreatePinnedToCore(
        proc_audio_data, 
        "Audio DSP",
        DSP_TASK_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1, // Verify this priority, should be low
        &xDSPTaskHandle,
        DSP_TASK_CORE
    );
    if ( xReturned != pdPASS )
    {
        ESP_LOGE(TAG, "Failed to create DSP task! Error: %d. Restarting ESP in 5 seconds...", xReturned);
        vTaskDelete(xDSPTaskHandle);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }
    else
    {
        ESP_LOGW(TAG, "Instantiated DSP task successfully!");
    }

    // Enable channels
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
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
