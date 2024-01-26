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
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

// Local libraries
#include <es8388.h>
#include <algo_common.h>
#include <peak_shift.h>
#include "esp_vocoder_main.h"
#include <Yin.h>

// Number of samples
#define HOP_SIZE (N_SAMPLES / 2) // Overlap 50%
#define HOP_BUFFER_SIZE_B (2 * HOP_SIZE * 4) // == length of rx/tx buffers (bytes)
// rx/tx buffers. Size doubled for L+R (even if only R is used)
int N = N_SAMPLES;

// I2S macros
#define I2S_SAMPLING_FREQ_HZ (40960) // Slightly lower, but more even frequency resolution
#define I2S_DOWNSHIFT (8) // 24-bit precision, can downshift safely by byte for FFT calcs
#define I2S_DOWNSHIFT_DIV (256) // Shorthand for 1 << 8

// FFT buffers
__attribute__((aligned(16))) float rx_FFT[N_SAMPLES * 2]; // Will be complex
__attribute__((aligned(16))) float tx_iFFT[N_SAMPLES * 2];
__attribute__((aligned(16))) float prev_rx_FFT[FFT_MOD_SIZE * 2]; // For inst freq calc
__attribute__((aligned(16))) float prev_tx_FFT[FFT_MOD_SIZE * 2];
// Peak shift buffers
float rx_FFT_mag[FFT_MOD_SIZE]; // Needed for peak shifting
// True envelope buffers
// Use tx_iFFT to save memory for envelope FFT/cepstrum buffers
float* env_FFT = tx_iFFT;
float* cepstrum_buff = tx_iFFT + N_SAMPLES;
// Allocate envelope buffer
float rx_env[FFT_MOD_SIZE];
float rx_env_inv[FFT_MOD_SIZE]; // Inverse for ratio calc

// Yin pitch detection
#define YIN_SAMPLES (HOP_SIZE) 
static Yin yin_s;
static float yinBuffPtr[YIN_SAMPLES / 2];
float yin_f0_est;

#define NOISE_THRESHOLD_DB  (14) // Empirical
#define SILENCE_RESET_COUNT (10) // every quarter second

// TX/RX buffers
#define DELAY_TAP_SIZE   (0) // No delay tap for now
// Full size will be delay tap size + N + hop size
#define FULL_BUFFER_SIZE (DELAY_TAP_SIZE + N_SAMPLES + HOP_SIZE)
int* rxBuffer;
int* txBuffer;

// I2S RX will fill the *end* of the RX Buffer
#define i2s_rx (rxBuffer + N_SAMPLES + DELAY_TAP_SIZE) 
// DSP RX uses start of buffer, which will include overlap + new data, but not delay tap
#define dsp_rx (rxBuffer + DELAY_TAP_SIZE)
// I2S TX will pull from the *beginning* of the TX buffer (excluding delay tap)
#define i2s_tx (txBuffer)
// DSP TX will fill in latter portion
#define dsp_tx (txBuffer + HOP_SIZE)

// Stream handles
i2s_chan_handle_t rx_handle, tx_handle;
// Divider for full size of hop buffer
#define RX_STREAM_BUFF_DIV (8)
#define RX_STREAM_BUFF_SIZE_B (HOP_BUFFER_SIZE_B / RX_STREAM_BUFF_DIV)

// Task handles
TaskHandle_t xDSPTaskHandle;
TaskHandle_t  xRxTaskHandle;
TaskHandle_t  xTxTaskHandle;
TaskHandle_t  xTaskStatsHandle;
TaskHandle_t  xModeSwitchTaskHandle;
// Stack sizes based empirically on high watermarks, with a little extra room just in case
// Should be revisited if changes are made
#define DSP_TASK_STACK_SIZE (15000u) // Watermark: 14368
#define DSP_TASK_CORE (1)
#define RX_TASK_STACK_SIZE (3500u) // Watermark: 3464
#define RX_TASK_CORE (0)
#define TX_TASK_STACK_SIZE (3500u) // Watermark: 3456
#define TX_TASK_CORE (0)
#define TASK_STATS_STACK_SIZE (3000u) // Watermark: 1564, but crashes otherwise
#define TASK_STATS_CORE (0)
#define MODE_SWITCH_TASK_STACK_SIZE (800u) // Watermark: 212
#define MODE_SWITCH_TASK_CORE (0)

#define DSP_TASK_PRIORITY (1U) // Just above idle
#define TX_TASK_PRIORITY (10U) // Higher due to it being blocked
#define RX_TASK_PRIORITY (10U) // Higher still since it will spend the most time blocked
#define TASK_STATS_PRIORITY (0U) // == IDLE PRIO
#define MODE_SWITCH_TASK_PRIORITY (5U) // A little higher, but should be quick

#define I2S_BUFFER_TIME_MS (1000.0 * HOP_SIZE / I2S_SAMPLING_FREQ_HZ) // Transmit/receive buffer time for I2S channels

// Sync events
EventGroupHandle_t RxSync, TxSync;
#define I2S_SYNC_BIT (0x1)
#define DSP_SYNC_BIT (0x2)
#define ALL_SYNC_BITS (0x3) // I2S Sync | DSP Sync

// Semaphores
SemaphoreHandle_t xModeSwitchMutex;
portMUX_TYPE crit_mux = portMUX_INITIALIZER_UNLOCKED;

// Lookup table for pitch shift factors
#define NUM_PITCH_SHIFTS (3) // Unity (root) done separately
// Pitch-shift for minor 7th where original sound is 5th
static const float PITCH_SHIFT_FACTORS[] = {
  // 1.0, // Original sound
  0.66667, // Lower Fourth ("Tonic")
  0.8, // Lower 6th flat (Minor third of new tonic, 2/3 * 6/5 = 4/5)
  1.2, // Minor Third (Flat seventh of new tonic)
};
static const float PITCH_SHIFT_GAINS[] = {
  // 1.0, // Original sound
  1.1, // Lower fourth
  1.0, // Lower sixth flat
  0.8, // Minor third
};

#define LOW_EFFECT_SHIFT  (0.5)
#define LOW_EFFECT_GAIN   (1.2)
#define HIGH_EFFECT_SHIFT (2.0)
#define HIGH_EFFECT_GAIN  (1.0)
#define PASSTHROUGH_SHIFT (1.0)
#define PASSTHROUGH_GAIN  (1.0)


typedef enum {
  MOD_AUTOTUNE,
  MOD_CHORUS,
  MOD_LOW,
  MOD_HIGH,
  MAX_VOCODER_MODE
} vocoder_mode_e;

// Mode switch config
#define GPIO_LED_1       (GPIO_NUM_22)
#define GPIO_LED_2       (GPIO_NUM_19)
#define GPIO_MODE_SWITCH (GPIO_NUM_36)
static vocoder_mode_e vocoder_mode;

// Stats trackers
static unsigned loop_count      = 0;
static float dsp_calc_time_sum  = 0;
static float num_peaks_sum      = 0;
static unsigned phase_reset_count = 0;
static unsigned rx_ovfl_hit     = 0;
static unsigned tx_ovfl_hit     = 0;
static unsigned mode_switch_isr_count = 0;

// Autotune lookup table
// Returns closest note in Hz
const float PARTIAL_LOOKUP_TABLE_HZ[] = {
  32.70, // C1
  34.65, // C#1
  36.71, // D1
  38.89, // D#1
  41.20, // E1
  43.65, // F1
  46.25, // F#1
  49.00, // G1
  51.91, // G#1
  55.00, // A1
  58.27, // A#1
  61.74, // B1
  65.41, // C2
  69.30, // C#2
  73.42, // D2
  77.78, // D#2
  82.41, // E2
  87.31, // F2
  92.50, // F#2
  98.00, // G2
  103.83, // G#2
  110.00, // A2
  116.54, // A#2
  123.47, // B2
  130.81, // C3
  138.59, // C#3
  146.83, // D3
  155.56, // D#3
  164.81, // E3
  174.61, // F3
  185.00, // F#3
  196.00, // G3
  207.65, // G#3
  220.00, // A3
  233.08, // A#3
  246.94, // B3
  261.63, // C4
  277.18, // C#4
  293.66, // D4
  311.13, // D#4
  329.63, // E4
  349.23, // F4
  369.99, // F#4
  392.00, // G4
  415.30, // G#4
  440.00, // A4
  466.16, // A#4
  493.88, // B4
  523.25, // C5
  554.37, // C#5
  587.33, // D5
  622.25, // D#5
  659.25, // E5
  698.46, // F5
  739.99, // F#5
  783.99, // G5
  830.61, // G#5
  880.00, // A5
  932.33, // A#5
  987.77, // B5
  1046.50, // C6
  1108.73, // C#6
  1174.66, // D6
  1244.51, // D#6
  1318.51, // E6
  1396.91, // F6
  1479.98, // F#6
  1567.98, // G6
  1661.22, // G#6
  1760.00, // A6
  1864.66, // A#6
  1975.53, // B6
  2093.00, // C7
  2217.46, // C#7
  2349.32, // D7
  2489.02, // D#7
  2637.02, // E7
  2793.83, // F7
  2959.96, // F#7
  3135.96, // G7
  3322.44, // G#7
  3520.00, // A7
  3729.31, // A#7
  3951.07, // B7 
};
#define NUM_PARTIALS (84) // 7 octaves, 12 notes per octave

// Filter for A Major scale for more pronounced effect
#define A_INDICES      (9) // mod 12 => 9
#define B_INDICES      (11)
#define CSHARP_INDICES (1)
#define D_INDICES      (2)
#define E_INDICES      (4)
#define FSHARP_INDICES (6)
#define GSHARP_INDICES (8)

#endif // __ESP_VOCODER_MAIN__