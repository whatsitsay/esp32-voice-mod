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
static int _fundamental_freq_idx = -1;
static uint8_t _peak_flag_arr[(FFT_MOD_SIZE / 8) + 1];
static float _inst_freq_arr[FFT_MOD_SIZE];
static float _peak_phase_arr[FFT_MOD_SIZE];
static float _exp_phase_incr[FFT_MOD_SIZE];

// Index correction LUT
// Enough space to approach asymptotes, plus origin
// static float _index_correction_lut[IDX_CORR_SIZE];
// Constants for ease-of-access later
// const int IDX_CORR_SIZE_CONST = IDX_CORR_SIZE;
// const int IDX_CORR_FUNDAMENTAL_CONST = IDX_CORR_FUNDAMENTAL;
const int NYQUIST_FREQ_IDX = N_SAMPLES / 2;

const char* TAG = "Peak Shift Algorithm";

void init_peak_shift_cfg(peak_shift_cfg_t* cfg)
{
  peak_shift_cfg = cfg;

  // Initialize expected phase increment per frame
  for (int k = 0; k < FFT_MOD_SIZE; k++)
  {
    _exp_phase_incr[k] = (k * TWO_M_PI / N_SAMPLES) * cfg->hop_size;
  }
}

void reset_phase_comp_arr(float* phase_comp)
{
  for (int i = 0; i < 2 * FFT_MOD_SIZE; i += 2)
  {
    // Set to polar unity (real = 1, imag = 0)
    phase_comp[i]   = 1;
    phase_comp[i+1] = 0;
  }
}

int find_local_peaks(void)
{
  // Reset counters and flags
  _num_peaks = 0;
  _fundamental_freq_idx = -1;
  memset(_peak_flag_arr, 0, sizeof(_peak_flag_arr));

  float* mag_arr = peak_shift_cfg->fft_mag_ptr;

  // track last peak index and running peak difference
  int max_peak_diff = 0;
  int last_peak_idx = -1;

  // Iterate over magnitude array for indicies within bounds of peak detection,
  // taking into acount comparisons with previous/next neighbors
  // Only iterate through half of FFT, as it will be reflected by midpoint (Nyquist freq)
  for (int i = 0; i <= NYQUIST_FREQ_IDX; i++)
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
    int highest_neighbor = MIN(NYQUIST_FREQ_IDX, i + num_neighbors);
    for (int j = lowest_neighbor; j < highest_neighbor; j++) {
      if (j == i) continue; // Skip index itself
      if (mag_arr[j] >= curr_mag) {
        is_peak = false; // Greater value in range, not a peak
        break; // Save cycles
      }
    }

    // Set flag in array accordingly
    SET_ARR_BIT(_peak_flag_arr, i, is_peak);

    // Move on if not peak
    if (!is_peak) continue;

    // Calculate instantaneous frequency of peak
    // Get phase of current and previous frames
    float peak_phase = get_idx_phase(peak_shift_cfg->fft_ptr, i);
    float prev_phase = get_idx_phase(peak_shift_cfg->fft_prev_ptr, i);

    // Better estimate actual frequency of peak using phase difference
    // with previous frame (back calculation only, as this is real-time)
    float phase_diff = fmodf(peak_phase - prev_phase - _exp_phase_incr[i], TWO_M_PI);
    // Wrap between pi and -pi by taking advantage of periodicity
    if (phase_diff > M_PI) phase_diff = -TWO_M_PI + phase_diff;

    // Correction is defined as the ratio of this phase diff over 2pi, times the bin frequency step
    float freq_diff = (phase_diff * peak_shift_cfg->bin_freq_step) / (TWO_M_PI);
    // Store inst frequency in array for later use during pitch shifting
    _inst_freq_arr[i] = (i * peak_shift_cfg->bin_freq_step) + freq_diff;
    _peak_phase_arr[i] = peak_phase;

    // Increment peak counter
    _num_peaks++;

    if (i < FUNDAMENTAL_FREQ_MAX_BIN) {
      // Check if fundamental frequency
      // Skip check if last idx not yet set
      if (last_peak_idx != -1) max_peak_diff = MAX(max_peak_diff, i - last_peak_idx);
      // Store last peak index
      last_peak_idx = i;
    }
  }

  // Calculate fundamental frequency from maximum peak difference / 2
  _fundamental_freq_idx = max_peak_diff;

  // Return number of peaks
  return _num_peaks;
}

float est_fundamental_freq(void)
{
  return _fundamental_freq_idx * peak_shift_cfg->bin_freq_step;
}

void print_local_peaks(void)
{
  ESP_LOGW(TAG, "%d peaks detected, fundamental frequency ~%.2f Hz",
           _num_peaks, est_fundamental_freq());
}

float _get_true_env_correction(int old_idx, int new_idx)
{
  float true_env_gain = peak_shift_cfg->true_env_ptr[new_idx] * peak_shift_cfg->inv_env_ptr[old_idx];
  return true_env_gain;
}

void shift_peaks(float shift_factor, float shift_gain, float true_env_scale)
{
  // First check if unity.
  // If so, simply add to output FFT
  // Can't use SIMD due to shift gain factor
  if (shift_factor == 1)
  {
    for (int i = 0; i < 2 * FFT_MOD_SIZE; i++)
    {
      peak_shift_cfg->fft_out_ptr[i] += peak_shift_cfg->fft_ptr[i] * shift_gain;
    }
    return;
  }

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
    int new_peak      = i + idx_shift;

    // Calc peak phase propagation
    // Get phase of previous frame output FFT at new peak index
    float prev_peak_phase = get_idx_phase(peak_shift_cfg->fft_out_prev_ptr, new_peak);
    // Convert peak instantaneous frequency back to radians
    float inst_freq_rad = inst_freq * M_PI / 180.0;
    // Next calculate phase propagation for ROI
    float prop_phase = prev_peak_phase + inst_freq_rad * peak_shift_cfg->hop_size;
    // Per-bin phase prop will lose the original bin's phase to customize per bin 
    // Also mod with 2pi
    float per_bin_prop_phase = fmodf(prop_phase - _peak_phase_arr[i], TWO_M_PI);
    // Convert to polar coordinates
    float phase_prop_real, phase_prop_imag;
    polar_to_complex(1, per_bin_prop_phase, &phase_prop_real, &phase_prop_imag);

    // Iterate through ROI
    // Increment by 2's for complex values
    for (int j = 2 * new_roi_start; j <= 2 * new_roi_end; j += 2)
    {
      // Check if boundary has been hit
      int new_idx = (j < 0)         ? -1 * j : // Reflect back along origin
                    (j > N_SAMPLES) ? N_SAMPLES - j : // Reflect along upper boundary (Nyquist * 2 = N_SAMPLES)
                    j; // Use index as-is
      bool hit_boundary = new_idx != j;
      // Store original index
      int orig_idx = j - (2 * idx_shift);
      
      // Set value at new ROI index as product of original index
      // value and phase propagation value
      // Index shift is doubled due to real+imag
      float orig_fft_real = peak_shift_cfg->fft_ptr[orig_idx];
      float orig_fft_imag = peak_shift_cfg->fft_ptr[orig_idx + 1];
      float prod_fft_real, prod_fft_imag;
      mult_complex(orig_fft_real,
                   orig_fft_imag,
                   phase_prop_real,
                   phase_prop_imag,
                   &prod_fft_real,
                   &prod_fft_imag);
      
      // If boundary has been hit, correct for conjugate reflection
      if (hit_boundary) prod_fft_imag *= -1;

      // Calculate true envelope correction based on peak freq shift
      // If new index is below the estimated fundamental, just apply unity gain instead
      // If it's equivalent, go half-and-half
      // FIXME: might be worth having some sort of linear slope when below fundamental, instead of hard cliff
      int new_bin_idx  = new_idx / 2;
      int orig_bin_idx = orig_idx / 2;
      float true_env_corr = true_env_scale * _get_true_env_correction(orig_bin_idx, new_bin_idx);
      true_env_corr += (1 - true_env_scale);

      // Add to output FFT at new index, now applying shift_gain and true envelope correction
      // to both real and imaginary components
      peak_shift_cfg->fft_out_ptr[new_idx]   += prod_fft_real * shift_gain * true_env_corr;
      peak_shift_cfg->fft_out_ptr[new_idx+1] += prod_fft_imag * shift_gain * true_env_corr;
    }
  }
}

