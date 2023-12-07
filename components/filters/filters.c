
/**
 * @file filters.c
 * @author Gabe Kaufman (gkaufman93@gmail.com)
 * @brief Implementation for various frequency-domain filters for use in effects
 * @version 0.2
 * @date 2023-10-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include "freertos/FreeRTOS.h"
#include "filters.h"
#include <algo_common.h>

// Params for true envelope calculation
static float* _fft_buff;
static float* _cepstrum_buff;
static int _sampling_freq;
static uint8_t _subsample_flags[(FFT_MOD_SIZE / 8) + 1];

void config_true_env_calc(float* fft_buff, float* cepstrum_buff, float sampling_freq_hz)
{
  _fft_buff      = fft_buff;
  _sampling_freq = sampling_freq_hz;
  _cepstrum_buff = (cepstrum_buff == NULL) ? (float *)calloc(CEPSTRUM_MOD_SIZE, sizeof(float)) : cepstrum_buff;
  configASSERT( _cepstrum_buff != NULL );
}

void calc_cepstrum(float* mag_log_ptr, float* cepstrum_ptr, float cutoff_freq)
{
  // First fill FFT with log magnitude
  for (int i = 0; i <= CEPSTRUM_LEN / 2; i++) {
    volatile float mag_val = mag_log_ptr[i];
    _fft_buff[2 * i] = mag_val; // Real component is log magnitude
    _fft_buff[2 * i + 1] = 0; // Zero imag (no phase)
  }

  // Mirror FFT
  fill_mirror_fft(_fft_buff, CEPSTRUM_LEN);

  // Take IFFT
  inv_fft(_fft_buff, CEPSTRUM_LEN);
  // Zero out imaginary components
  dsps_mulc_f32(_fft_buff+1, _fft_buff+1, CEPSTRUM_LEN, 0, 2, 2);

  // Window for cutoff frequency using simple low-pass filter
  volatile int cutoff_idx = (int)roundf(2 * cutoff_freq * (CEPSTRUM_LEN / (float)_sampling_freq));
  // Halve value at cutoff frequency itself
  _fft_buff[2 * cutoff_idx] *= 0.5;
  // Apply low-pass filter by setting all higher frequencies to zero
  int lpf_zero_start = cutoff_idx + 1;
  int lpf_zero_size  = CEPSTRUM_MOD_SIZE - lpf_zero_start; // Up to Nyquist
  memset(_fft_buff + 2*lpf_zero_start, 0, 2 * lpf_zero_size * sizeof(float));

  // Fill mirror
  fill_mirror_fft(_fft_buff, CEPSTRUM_LEN);

  // Take FFT
  calc_fft(_fft_buff, CEPSTRUM_LEN);

  // Fill cepstrum array with real values only
  for (int i = 0; i <= CEPSTRUM_LEN / 2; i++) {
    cepstrum_ptr[i] = _fft_buff[2 * i];
  }
}

void calc_true_envelope(float* mag_log_ptr, float* env_ptr, float cutoff_freq)
{
  // Clear subsample flag array
  memset(_subsample_flags, 0, sizeof(_subsample_flags));

  // Start by subsampling by using max of each 2
  // NOTE: this is based on rough interpretation of the "efficient true envelope" paper, but may not be accurate
  for (int i = 0; i < CEPSTRUM_LEN / 2; i++) {
    // Select the index which has the greater value of the two
    float l_value = mag_log_ptr[2*i];
    float r_value = mag_log_ptr[2*i + 1];
    int sel_idx = (l_value >= r_value) ? 2*i : 2*i + 1;

    // Set envelope value to selected index
    env_ptr[i] = mag_log_ptr[sel_idx];
     
    // Set subsample indication bits to track for interpolation later
    SET_ARR_BIT(_subsample_flags, sel_idx, 1);
  }

  // Last point is always selected (Nyquist)
  env_ptr[CEPSTRUM_LEN / 2] = mag_log_ptr[N_SAMPLES / 2];
  SET_ARR_BIT(_subsample_flags, N_SAMPLES / 2, 1);

  // Iterate a finite amount of times to fine-tune envelope
  for (int j = 0; j < TRUE_ENV_NUM_ITERATIONS; j++) {
    // Reselect env values based on max of cepstrum and original log spectrum
    // If first iteration, use envelope as-is
    for (int i = 0; i < CEPSTRUM_MOD_SIZE; i++) {
      _cepstrum_buff[i] = (j > 0) ? MAX(env_ptr[i], _cepstrum_buff[i]) : env_ptr[i];
    }

    // Calculate cepstrum
    calc_cepstrum(_cepstrum_buff, _cepstrum_buff, cutoff_freq);
  }

  // Iterate through envelope, inserting values at their selected indicies
  for (int i = 0; i < FFT_MOD_SIZE; i ++) {
    // Check whether actual value was subsampled or neighbor
    bool val_used = GET_ARR_BIT(_subsample_flags, i); 

    // If value used, insert current cepstrum magnitude. Otherwise, insert 0 for now
    env_ptr[i] = (val_used) ? _cepstrum_buff[i / 2] : 0;
  }

  // Iterate again for interpolation/extrapolation of remaining values
  for (int i = 0; i < FFT_MOD_SIZE; i++) {
    // Skip if already selected
    if (GET_ARR_BIT(_subsample_flags, i)) continue;

    // Get indices for interpolation
    int idx_0, idx_1;

    // Account for edge cases
    switch (i) {
      case 0: {
        // First value
        // Get next two relevant values and extrapolate
        // First one should always be i+1, as it is the first pair
        idx_0 = i+1;
        // Next index can be variable, use subsample flags
        // Only possible to be max of 2 indices off
        idx_1 = (GET_ARR_BIT(_subsample_flags, idx_0+1)) ? idx_0+1 : idx_0+2;
        break;
      }
      case N_SAMPLES / 2: {
        // Last value
        // Get previous two values and extrapolate
        // Penultimate value should be valid, given selection in pairs
        idx_1 = i-1;
        // Other index will be the last selected index before this one
        // Again, only possible ot be max 2 indices off
        idx_0 = (GET_ARR_BIT(_subsample_flags, idx_1-1)) ? idx_1-1 : idx_1-2;
        break;
      }
      default: {
        // All other indices, check array in both directions
        idx_0 = (GET_ARR_BIT(_subsample_flags, i-1)) ? i-1 : i-2;
        idx_1 = (GET_ARR_BIT(_subsample_flags, i+1)) ? i+1 : i+2;

        // Check here to make sure max distance is less than 3
        configASSERT( idx_1 - idx_0 <= 3 );
        break;
      }
    }
    // Interpolate/extrapolate based on idx0 and idx1
    volatile float interpolation_val = interpolate_val(i, idx_1, idx_0, env_ptr);
    env_ptr[i] = interpolation_val;
  }
}