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
  // Reset peak counter and flags
  _num_peaks = 0;
  memset(_peak_flag_arr, 0, sizeof(_peak_flag_arr));

  float* mag_arr = peak_shift_cfg->fft_mag_ptr;
  int num_neighbors = 0;

  // Iterate over magnitude array for indicies within bounds of peak detection,
  // taking into acount comparisons with previous/next neighbors
  // Only iterate through half of FFT, as it will be reflected by midpoint (Nyquist freq)
  for (int i = 0; i <= N_SAMPLES / 2; i++)
  {
    float curr_mag = mag_arr[i];
    // Peak is defined as a frequency whose magnitude is greater than all of its synchornized neighbors
    bool is_peak = true;
    // Iterate through neighbors
    // Cap sync at boundaries of FFT
    int lowest_neighbor  = MAX(0, i - num_neighbors);
    int highest_neighbor = MIN(N_SAMPLES / 2, i + num_neighbors);
    for (int j = lowest_neighbor; j < highest_neighbor; j++) {
      if (j == i) continue; // Skip index itself
      if (mag_arr[j] > curr_mag) {
        is_peak = false; // Greater value in range, not a peak
        break; // Save cycles
      }
    }

    // Set flag in array accordingly
    SET_ARR_BIT(_peak_flag_arr, i, is_peak);

    // Increment counter if peak
    if (is_peak) _num_peaks++;

    // Based on the 'CHALLENGE' project (Bargun, Serafin, Erkut 2023), instead of just using midpoints, correlate
    // adjacent bins only based on band. Lower frequency => less neighbors
    // Current math is for every band interval, number of adjacent bins to sync goes increments
    // by 1
    // Thus, increment num neighbors on last bin of band
    if (i % BAND_DIV_NBINS == BAND_DIV_NBINS - 1) num_neighbors++;
  }

  // // Iterate through all peaks and populate other data values
  // // Limits longer calculations, like phase, to just actual chosen peaks
  // SPLAY_FOREACH(curr_peak, peak_tree, &_head)
  // {
  //   // Store bounds
  //   int num_neighbors = i / BAND_DIV_NBINS;
  //   left_bound = i - num_neighbors;
  //   // For right bound, cap at Nyquist bin
  //   right_bound = MIN(i + num_neighbors, N_SAMPLES/2);

  //   // Store phase
  //   peak_phase = get_idx_phase(peak_shift_cfg->fft_ptr, i);

  //   // Better estimate actual frequency of peak using phase difference
  //   // with previous frame (back calculation only, as this is real-time)
  //   float phase_diff = peak_phase - get_idx_phase(peak_shift_cfg->fft_prev_ptr, i);
  //   // Bound between pi and -pi
  //   phase_diff = MIN(M_PI, phase_diff);
  //   phase_diff = MAX(-M_PI, phase_diff);
  //   // Correction is defined as the ratio of this phase diff over 2pi, times the bin frequency step
  //   float freq_diff = (phase_diff * peak_shift_cfg->bin_freq_step) / (2 * M_PI);
  //   // Correct for instantaneous frequency estimate
  //   inst_freq = (i * peak_shift_cfg->bin_freq_step) + freq_diff;
  // }

  // // Return number of peaks
  return _num_peaks;
}

void print_local_peaks(void)
{
  ESP_LOGW(TAG, "%d peaks detected", _num_peaks);
  // TODO: maybe have some way to display peak indicies
  // if (_num_peaks == 0) return;

  // char peak_str[1024];
  // // Copy in initial string. Can be done without size, as it is much less than buffer length
  // sprintf(peak_str, "%d Largest Peak Values (Hz): ", MAX_PRINT_PEAKS);

  // struct peak_node_t* curr_peak = SPLAY_MIN(peak_tree, &_head);
  // for (int i = 0; i < _num_peaks; i++)
  // {
  //   char peak_val[50];
  //   // Iterate through tree until last few entries
  //   // Need to iterate this way given unidirectional nature of tree
  //   // (I.e., can only traverse with SPLAY_NEXT to next higher value)
  //   if (i > _num_peaks - 1 - MAX_PRINT_PEAKS)
  //   {
  //     configASSERT(curr_peak != NULL);
  //     int peak_freq_hz = roundf(inst_freq);
  //     float peak_mag_db = mag_db;
  //     int left_bound = left_bound * peak_shift_cfg->bin_freq_step;
  //     int right_bound = right_bound * peak_shift_cfg->bin_freq_step;
  //     sprintf(peak_val, "%d (%.2f dB, %d-%d) ", peak_freq_hz, peak_mag_db, left_bound, right_bound);
  //     strcat(peak_str, peak_val);
  //   }

  //   curr_peak = SPLAY_NEXT(peak_tree, &_head, curr_peak);
  // }

  // ESP_LOGW(TAG, "%s", peak_str);
}

void shift_peaks_int(float shift_factor, float shift_gain, float* run_phase_comp_ptr)
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

    // Store phase
    float peak_phase = get_idx_phase(peak_shift_cfg->fft_ptr, i);

    // Better estimate actual frequency of peak using phase difference
    // with previous frame (back calculation only, as this is real-time)
    float phase_diff = peak_phase - get_idx_phase(peak_shift_cfg->fft_prev_ptr, i);
    // Bound between pi and -pi
    phase_diff = MIN(M_PI, phase_diff);
    phase_diff = MAX(-M_PI, phase_diff);
    // Correction is defined as the ratio of this phase diff over 2pi, times the bin frequency step
    float freq_diff = (phase_diff * peak_shift_cfg->bin_freq_step) / (2 * M_PI);
    // Correct for instantaneous frequency estimate
    float inst_freq = (i * peak_shift_cfg->bin_freq_step) + freq_diff;

    // Calculate the desired change in frequency based on the peak instantaneous
    // frequency and the shift factor
    float delta_f_raw = (shift_factor - 1) * inst_freq;

    // Round to get the index shift for this ROI
    int idx_shift = (int)roundf(delta_f_raw / peak_shift_cfg->bin_freq_step);
    int new_roi_start = left_bound + idx_shift;
    int new_roi_end   = right_bound + idx_shift;

    // Correct frequency change to be this integer increment
    // Should be uncorrected for sampling frequency
    float delta_f = (2 * M_PI * idx_shift) / (1.0 * N_SAMPLES);

    // Calculate phase compensation for ROI based on this delta freq
    // NOTE: This is pulled from the paper directly, but doesn't quite make
    // sense. I suppose it's representing the phase shift as a function of frequency?
    float phase_comp_real = cosf(delta_f * peak_shift_cfg->hop_size);
    float phase_comp_imag = sinf(delta_f * peak_shift_cfg->hop_size);

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

      // TODO: may need to double-check this in the case of overlapping sections,
      // especially if there are multiple ROI in one section. This would effectively
      // add up the phase compensation. Does this make sense? Or should it be averaged/
      // use max?
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

