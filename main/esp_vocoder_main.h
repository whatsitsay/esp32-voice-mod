/**
 * @file esp_vocoder_main.c
 * @author Gabriel Kaufman (gkaufman93@gmail.com)
 * @brief Header for main module of ESP32 phase vocoder
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

#ifndef __ESP_VOCODER_MAIN__
#define __ESP_VOCODER_MAIN__

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// Local libraries
#include <es8388.h>
#include <algo_common.h>
#include <peak_shift.h>
#include "esp_vocoder_main.h"

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
// Peak shift buffers
__attribute__((aligned(16))) float rx_FFT_mag[FFT_MOD_SIZE]; // Needed for peak shifting
__attribute__((aligned(16))) float run_phase_comp[2 * FFT_MOD_SIZE]; // Cumulative phase compensation buffer
float prev_rx_phase[FFT_MOD_SIZE]; // Needed for input instantaneous angle calc
float prev_tx_phase[FFT_MOD_SIZE]; // Needed for output phase calc

#define NOISE_THRESHOLD_DB (28) // Empirical
#define SILENCE_RESET_COUNT (10) // ~every half second

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
TaskHandle_t xDSPTaskHandle;
TaskHandle_t  xRxTaskHandle;
TaskHandle_t  xTxTaskHandle;
TaskHandle_t  xTaskStatsHandle;
TaskHandle_t  xModeSwitchTaskHandle;
// Stack sizes based empirically on high watermarks, with a little extra room just in case
// Should be revisited if changes are made
#define DSP_TASK_STACK_SIZE (15500u) // Watermark: 15284
#define DSP_TASK_CORE (0)
#define RX_TASK_STACK_SIZE (3600u) // Watermark: 3464
#define RX_TASK_CORE (1)
#define TX_TASK_STACK_SIZE (3600u) // Watermark: 3456
#define TX_TASK_CORE (1)
#define TASK_STATS_STACK_SIZE (3000u) // Watermark: 1164, but crashes on anything lower
#define TASK_STATS_CORE (0)
#define MODE_SWITCH_TASK_CORE (1)
#define MODE_SWITCH_TASK_STACK_SIZE (1024u) // Check watermark!

#define DSP_TASK_PRIORITY (1U) // Just above idle
#define TX_TASK_PRIORITY (10U) // Higher due to it being blocked
#define RX_TASK_PRIORITY (10U) // Higher still since it will spend the most time blocked
#define TASK_STATS_PRIORITY (0U) // == IDLE PRIO
#define MODE_SWITCH_TASK_PRIORITY (5U) // A little higher, but should be quick

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
SemaphoreHandle_t xDbgMutex, xModeSwitchMutex;

#define PLOT_LEN (128)
__attribute__((aligned(16))) float tx_FFT_mag[PLOT_LEN]; // For debug only

// Lookup table for pitch shift factors
#define NUM_PITCH_SHIFTS (4)
// Pitch-shift for minor 7th where original sound is 5th
static const float PITCH_SHIFT_FACTORS[] = {
  1.0, // Original sound
  0.66667, // Lower Fourth ("Tonic")
  0.8, // Lower 6th flat (Minor third of new tonic, 2/3 * 6/5 = 4/5)
  1.2, // Minor Third (Flat seventh of new tonic)
};
static const float PITCH_SHIFT_GAINS[] = {
  1.0, // Original sound
  1.1, // Lower fourth
  1.0, // Lower sixth flat
  0.8, // Minor third
};

#define LOW_EFFECT_SHIFT  (0.75)
#define LOW_EFFECT_GAIN   (1.1)
#define HIGH_EFFECT_SHIFT (1.5)
#define HIGH_EFFECT_GAIN  (1.0)
#define PASSTHROUGH_SHIFT (1.0)
#define PASSTHROUGH_GAIN  (1.0)


typedef enum {
  MOD_CHORUS,
  MOD_LOW,
  MOD_HIGH,
  PASSTHROUGH,
  MAX_VOCODER_MODE
} vocoder_mode_e;

// Mode switch config
#define GPIO_LED_1       (GPIO_NUM_22)
#define GPIO_LED_2       (GPIO_NUM_19)
#define GPIO_MODE_SWITCH (GPIO_NUM_36)
static vocoder_mode_e vocoder_mode = MOD_CHORUS;

// Stats trackers
static unsigned loop_count      = 0;
static float dsp_calc_time_sum  = 0;
static float num_peaks_sum      = 0;
static unsigned phase_reset_count = 0;
static unsigned rx_ovfl_hit     = 0;
static unsigned tx_ovfl_hit     = 0;
static unsigned mode_switch_isr_count = 0;

#endif // __ESP_VOCODER_MAIN__