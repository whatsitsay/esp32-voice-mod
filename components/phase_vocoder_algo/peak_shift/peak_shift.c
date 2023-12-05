/**
 * @file peak_shift.c
 * @author Gabriel Kaufman (whatsitsay)
 * @brief Implementation for peak-based pitch-shifting library
 * 
 * Algorithm based on the paper "New Phase-Vocoder Techniques for Pitch-Shifting,
 * Harmonizing, and Other Exotic Effects" by Jean Laroche and Mark Dolson
 * @version 0.1
 * @date 2023-09-29
 * 
 */

#include "peak_shift.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include <algo_common.h>

static peak_shift_cfg_t* peak_shift_cfg;

// Counter and flag arrays
static unsigned _num_peaks = 0;
static float _fundamental_freq = -1;
static uint8_t _peak_flag_arr[(FFT_MOD_SIZE / 8) + 1];
static float _inst_freq_arr[FFT_MOD_SIZE];

const char* TAG = "Peak Shift Algorithm";

void init_peak_shift_cfg(peak_shift_cfg_t* cfg)
{
  peak_shift_cfg = cfg;
}

void reset_phase_comp_arr(float* run_phase_comp_ptr)
{
  // Increment by two for real + imag
  for (int i = 0; i <= FFT_MOD_SIZE * 2; i += 2)
  {
    run_phase_comp_ptr[i]   = 1;
    run_phase_comp_ptr[i+1] = 0;
  }
}

int find_local_peaks(void)
{
  // Reset counters and flags
  _num_peaks = 0;
  _fundamental_freq = -1;
  memset(_peak_flag_arr, 0, sizeof(_peak_flag_arr));

  float* mag_arr = peak_shift_cfg->fft_mag_ptr;

  // track last peak index and running peak difference
  int max_peak_diff = 0;
  int last_peak_idx = -1;

  // Iterate over magnitude array for indicies within bounds of peak detection,
  // taking into acount comparisons with previous/next neighbors
  // Only iterate through half of FFT, as it will be reflected by midpoint (Nyquist freq)
  for (int i = 0; i <= N_SAMPLES / 2; i++)
  {
    float curr_mag = mag_arr[i];
    // Peak is defined as a frequency whose magnitude is greater than all of its synchornized neighbors
    bool is_peak = true;
    // Based on the 'CHALLENGE' project (Bargun, Serafin, Erkut 2023), instead of just using midpoints, correlate
    // adjacent bins only based on band. Lower frequency => less neighbors
    // Current math is for every band interval, number of adjacent bins to sync goes increments
    // by 1
    int num_neighbors = i / BAND_DIV_NBINS;
    // Iterate through neighbors
    // Cap sync at boundaries of FFT
    int lowest_neighbor  = MAX(0, i - num_neighbors);
    int highest_neighbor = MIN(N_SAMPLES / 2, i + num_neighbors);
    for (int j = lowest_neighbor; j < highest_neighbor; j++) {
      if (j == i) continue; // Skip index itself
      if (mag_arr[j] >= curr_mag) {
        is_peak = false; // Greater value in range, not a peak
        break; // Save cycles
      }
    }

    // Set flag in array accordingly
    SET_ARR_BIT(_peak_flag_arr, i, is_peak);

    if (is_peak) {
      // Calculate instantaneous frequency of peak
      // Get phase of current and previous frames
      float peak_phase = get_idx_phase(peak_shift_cfg->fft_ptr, i);
      float prev_phase = get_idx_phase(peak_shift_cfg->fft_prev_ptr, i);

      // Better estimate actual frequency of peak using phase difference
      // with previous frame (back calculation only, as this is real-time)
      float phase_diff = peak_phase - prev_phase;
      // Bound between pi and -pi by taking advantage of periodicity
      phase_diff = (phase_diff > M_PI) ? -2 * M_PI + phase_diff : // Wraparound to -pi
                   (phase_diff < -M_PI) ? 2 * M_PI + phase_diff : // Wraparound to +pi
                   phase_diff;                                    // Within range, use as-is
      // Correction is defined as the ratio of this phase diff over 2pi, times the bin frequency step
      float freq_diff = (phase_diff * peak_shift_cfg->bin_freq_step) / (2 * M_PI);
      // Store inst frequency in array for later use during pitch shifting
      _inst_freq_arr[i] = (i * peak_shift_cfg->bin_freq_step) + freq_diff;

      // Increment peak counter
      _num_peaks++;

      // Check if fundamental frequency
      // Skip check if last idx not yet set
      if (last_peak_idx != -1) max_peak_diff = MAX(max_peak_diff, i - last_peak_idx);
      // Store last peak index
      last_peak_idx = i;
    }
  }

  // Calculate fundamental frequency from maximum peak difference
  _fundamental_freq = max_peak_diff * peak_shift_cfg->bin_freq_step;

  // Return number of peaks
  return _num_peaks;
}

void print_local_peaks(void)
{
  ESP_LOGW(TAG, "%d peaks detected, fundamental frequency ~%.2f Hz",
           _num_peaks, _fundamental_freq);
}

float est_fundamental_freq(void)
{
  return _fundamental_freq;
}

void shift_peaks(float shift_factor, float shift_gain, float* run_phase_comp_ptr)
{
  // Iterate through peak flag array
  for (int i = 0; i < FFT_MOD_SIZE; i++)
  {

    // If not a peak, skip
    if (0 == GET_ARR_BIT(_peak_flag_arr, i)) continue;

    // Get left/right bound, phase and instantaneous frequency for idx
    int num_neighbors = i / BAND_DIV_NBINS;
    int left_bound = i - num_neighbors;
    // For right bound, cap at Nyquist bin
    int right_bound = MIN(i + num_neighbors, N_SAMPLES/2);

    // Get buffered instantaneous frequency
    float inst_freq = _inst_freq_arr[i];

    // Calculate the desired change in frequency based on the peak instantaneous
    // frequency and the shift factor
    float delta_f = (shift_factor - 1) * inst_freq;

    // Round to get the index shift for this ROI
    int idx_shift = (int)roundf(delta_f / peak_shift_cfg->bin_freq_step);
    int new_roi_start = left_bound + idx_shift;
    int new_roi_end   = right_bound + idx_shift;

    // Calculate phase compensation for ROI based on freq shift
    float phase_comp_angle = fmod(delta_f * peak_shift_cfg->hop_size, 2 * M_PI);
    float phase_comp_real  = cosf(phase_comp_angle);
    float phase_comp_imag  = sinf(phase_comp_angle);

    // Iterate through ROI
    // Increment by 2's for complex values
    for (int j = 2 * new_roi_start; j <= 2 * new_roi_end; j += 2)
    {
      // Check if boundary has been hit
      int new_idx = (j < 0)           ? -1 * j : // Reflect back along origin
                    (j > N_SAMPLES) ? (2 * N_SAMPLES) - j : // Reflect along upper boundary
                    j; // Use index as-is
      bool hit_boundary = new_idx != j;

      float* run_phase_comp_real = &run_phase_comp_ptr[new_idx];
      float* run_phase_comp_imag = &run_phase_comp_ptr[new_idx+1];
      
      // First multiply current frame phase compensation with running product
      // Product is *cumulative* between frames
      float prev_run_phase_comp_real = *run_phase_comp_real;
      float prev_run_phase_comp_imag = *run_phase_comp_imag;
      mult_complex(prev_run_phase_comp_real,
                  prev_run_phase_comp_imag,
                  phase_comp_real,
                  phase_comp_imag,
                  run_phase_comp_real,
                  run_phase_comp_imag);
      
      // Set value at new ROI index as product of original index
      // value and cumulative phase compensation
      // Index shift is doubled due to real+imag
      float orig_fft_real = peak_shift_cfg->fft_ptr[j - (2 * idx_shift)];
      float orig_fft_imag = peak_shift_cfg->fft_ptr[j + 1 - (2 * idx_shift)];
      float prod_fft_real, prod_fft_imag;
      mult_complex(orig_fft_real,
                   orig_fft_imag,
                   *run_phase_comp_real,
                   *run_phase_comp_imag,
                   &prod_fft_real,
                   &prod_fft_imag);
      
      // If boundary has been hit, correct for conjugate reflection
      if (hit_boundary) prod_fft_imag *= -1;

      // Add to output FFT at new index, now applying shift_gain to both real and imaginary components
      peak_shift_cfg->fft_out_ptr[new_idx]   += prod_fft_real * shift_gain;
      peak_shift_cfg->fft_out_ptr[new_idx+1] += prod_fft_imag * shift_gain;
    }
  }
}

