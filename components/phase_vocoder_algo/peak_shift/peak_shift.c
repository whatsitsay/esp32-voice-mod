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
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include "algo_common.h"

#define NUM_NEIGHBORS (2) // For peak detection

static peak_shift_cfg_t* peak_shift_cfg;
static peak_data_t peak_data[MAX_PEAKS];
static int num_peaks;

const char* TAG = "Peak Shift Algorithm";

void init_peak_shift_cfg(peak_shift_cfg_t* cfg)
{
  peak_shift_cfg = cfg;
  num_peaks = 0;
}

void reset_phase_comp_arr(float* run_phase_comp_ptr)
{
  // Increment by two for real + imag
  // However, *don't* double size as we only need the first half
  for (int i = 0; i <= peak_shift_cfg->num_samples; i += 2)
  {
    run_phase_comp_ptr[i]   = 1;
    run_phase_comp_ptr[i+1] = 0;
  }
}

int find_local_peaks(void)
{
  // Init running counter of number of peaks
  num_peaks = 0;

  // Set var for num samples, for ease of reference
  int num_samples = peak_shift_cfg->num_samples;

  // Fill magnitude buffer
  // Use half size to save memory/cycles (given it's for real signal)
  // +1 for midpoint (Nyquist freq)
  calc_fft_mag_db(peak_shift_cfg->fft_ptr, peak_shift_cfg->fft_mag_ptr, num_samples / 2 + 1);
  float* mag_arr = peak_shift_cfg->fft_mag_ptr;

  // Iterate over magnitude array for indicies within bounds of peak detection,
  // taking into acount comparisons with previous/next neighbors
  // Only iterate through half of FFT, as it will be reflected by midpoint (Nyquist freq)
  for (int i = NUM_NEIGHBORS; i <= peak_shift_cfg->num_samples / 2 - NUM_NEIGHBORS; i++)
  {
    float check_val = mag_arr[i];
    // First ensure check val is over threshold, to prevent noise
    if (check_val < PEAK_THRESHOLD_DB) continue;
    // Based on the paper used for the algorithm, a "peak" is defined as an index
    // whose magnitude is larger than it's two neighbors in each direction (left
    // and right). Supposedly, this should be good enough in practice
    if (check_val < mag_arr[i-2]) continue;
    if (check_val < mag_arr[i-1]) continue;
    if (check_val < mag_arr[i+1]) continue;
    if (check_val < mag_arr[i+2]) continue;

    // Ensure we haven't reached the max number of peaks
    // Otherwise, throw error by returning -1
    if (num_peaks >= MAX_PEAKS) {
      ESP_LOGE(TAG, "Reached maximum number of peaks!");
      return -1;
    }

    // If past those checks, value is local maxima
    // Store index in peak data array
    peak_data[num_peaks].idx = i;

    // Store bounds
    if (num_peaks == 0) 
    {
      // For first peak, left bound will be first index
      // Right bound will be stored later on
      peak_data[num_peaks].left_bound = 0;
    }
    else 
    {
      // Each region of influence (ROI) will be defined as between the midpoints
      // of each peak
      int midpoint = (peak_data[num_peaks].idx + peak_data[num_peaks - 1].idx) / 2;
      // Store previous peak right bound as midpoint
      peak_data[num_peaks - 1].right_bound = midpoint;
      // Store current peak left bound as midpoint+1
      // Right bound will be stored later on
      peak_data[num_peaks].left_bound = midpoint + 1;
    }

    // Store phase
    peak_data[num_peaks].phase = get_idx_phase(peak_shift_cfg->fft_ptr, i);

    // Better estimate actual frequency of peak using phase difference
    // with previous frame (back calculation only, as this is real-time)
    float phase_diff = peak_data[num_peaks].phase - get_idx_phase(peak_shift_cfg->fft_prev_ptr, i);
    // Bound between pi and -pi
    phase_diff = MIN(M_PI, phase_diff);
    phase_diff = MAX(-M_PI, phase_diff);
    // Correction is defined as the ratio of this phase diff over 2pi, times the bin frequency step
    float freq_diff = (phase_diff * peak_shift_cfg->bin_freq_step) / (2 * M_PI);
    // Correct for instantaneous frequency estimate
    peak_data[num_peaks].inst_freq = (i * peak_shift_cfg->bin_freq_step) + freq_diff;

    // Increment num_peaks
    num_peaks++;
  }

  // If at least one peak, store last rightmost bound as rightmost index
  // Again, this will be the midpoint due to reflection
  if (num_peaks > 0)
  {
    peak_data[num_peaks-1].right_bound = (peak_shift_cfg->num_samples/2);
  }

  // Return number of peaks
  return num_peaks;
}

void print_local_peaks(void)
{
  ESP_LOGW(TAG, "%d peaks detected", num_peaks);
  if (num_peaks == 0) return;

  char peak_str[1024] = "Peak Values (Hz): ";
  
  for (int i = 0; i < num_peaks; i++) 
  {
    char peak_val[50];
    int peak_freq_hz = roundf(peak_data[i].inst_freq);
    int left_bound = peak_data[i].left_bound * peak_shift_cfg->bin_freq_step;
    int right_bound = peak_data[i].right_bound * peak_shift_cfg->bin_freq_step;
    sprintf(peak_val, "%d (%d,%d) ", peak_freq_hz, left_bound, right_bound);
    strcat(peak_str, peak_val);
  }

  ESP_LOGW(TAG, "%s", peak_str);
}

void shift_peaks_int(float shift_factor, float* run_phase_comp_ptr)
{
  int num_samples = peak_shift_cfg->num_samples;

  // Iterate through all ROI
  for (int i = 0; i < num_peaks; i++)
  {
    // Calculate the desired change in frequency based on the peak instantaneous
    // frequency and the shift factor
    float delta_f_raw = (shift_factor - 1) * peak_data[i].inst_freq;

    // Round to get the index shift for this ROI
    int idx_shift = (int)roundf(delta_f_raw / peak_shift_cfg->bin_freq_step);
    int new_roi_start = peak_data[i].left_bound + idx_shift;
    int new_roi_end   = peak_data[i].right_bound + idx_shift;

    // Cap at actual boundaries if peak not actually surpassing boundary
    // TODO: may be worth setting some threshold value...
    // int new_peak_idx  = peak_data[i].idx + idx_shift;
    // if (new_roi_start < 0 && new_peak_idx >= 0) {
    //   new_roi_start = 0;
    // }
    // if (new_roi_end   > num_samples/2 && new_peak_idx <= num_samples/2) {
    //   new_roi_end = num_samples / 2;
    // }

    // Correct frequency change to be this integer increment
    // Should be uncorrected for sampling frequency
    float delta_f = (2 * M_PI * idx_shift) / (1.0 * num_samples);

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
                    (j > num_samples) ? (2 * num_samples) - j : // Reflect along upper boundary
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

      // Add to output FFT at new index
      peak_shift_cfg->fft_out_ptr[new_idx]   += prod_fft_real;
      peak_shift_cfg->fft_out_ptr[new_idx+1] += prod_fft_imag;
    }
  }
}

