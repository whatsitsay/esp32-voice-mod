/**
 * @file peak_shift.c
 * @author Gabriel Kaufman (whatsitsay)
 * @brief Header for peak-based pitch-shifting library
 * 
 * Algorithm based on the paper "New Phase-Vocoder Techniques for Pitch-Shifting,
 * Harmonizing, and Other Exotic Effects" by Jean Laroche and Mark Dolson
 * @version 0.1
 * @date 2023-09-29
 * 
 */

#ifndef __PEAK_SHIFT__
#define __PEAK_SHIFT__

#include <algo_common.h>

// Minimum threshold to consider magnitude a peak
// Value is empirical, based on testing of sound quality with peak-finding algorithm and calc time
#define MAX_PRINT_PEAKS (5) // To prevent overloading
#define BAND_DIV_NBINS (7) // Number of bins for each 'band' when calculating peaks
#define FUNDAMENTAL_FREQ_MAX_BIN (150) // Maximum bin for estimating the fundamental frequency (empirical)

typedef struct {
  int hop_size;               // Hop size of analysis
  float bin_freq_step;        // Frequency increment per bin of FFT array (sampling freq/N)
  float* fft_ptr;             // Pointer to input FFT array of current frame (size 2*N)
  float* fft_prev_ptr;        // Pointer to input FFT array of previous frame (size N+2)
  float* fft_mag_ptr;         // Pointer to input FFT magnitude array of current frame (size N/2+1)
  float* fft_out_ptr;         // Pointer to output FFT (size 2*N)
  float* fft_out_prev_ptr;    // Pointer to output FFT for previous frame
  float* true_env_ptr;        // Pointer to true envelope buffer (calculated externally)
  float* inv_env_ptr;         // Pointer to inverse of true envelope (i.e. 1/true_env above)
} peak_shift_cfg_t;

/**
 * @brief Store peak shift algorithm configuration and pointers
 * 
 * Note that all arrays, aside from peak_data, need to be instantiated *externally*
 * 
 * @param cfg - Pointer to struct containing configuration data and pointers to
 *              relevant arrays
 */
void init_peak_shift_cfg(peak_shift_cfg_t* cfg);

// TODO fill docstring
void reset_phase_comp_arr(float* phase_comp);

/**
 * @brief Locate peaks within FFT magnitude plot
 * 
 * Iterates through FFT magnitude plot (stored in config) and finds relative
 * maxima, defined loosely as an index where the magnitude is larger than it's
 * two neighbors in either direction.
 * 
 * It then stores the following in an array:
 * 1) Index of peak
 * 2) Left bound of peak ROI (region of influence)
 * 3) Right bound of peak ROI
 * 4) Instantaneous frequency of idx, estimated with FFT of previous frame
 * 5) Phase of the peak
 * 
 * @return Number of peaks found in magnitude array, -1 if MAX was surpassed
 */
int find_local_peaks(void);

/**
 * @brief Print local peaks logged during "find_local_peaks" method
 * 
 */
void print_local_peaks(void);

/**
 * @brief Estimate the fundamental frequency from the peak array
 * 
 * Estimated as the max difference (in Hz) between two peaks
 * Peaks should be found *first*
 * 
 * @return float - Fundamental frequency, or -1 if error
 */
float est_fundamental_freq(void);


/**
 * @brief Shift peaks by integer number of frequency bins
 * 
 * Based on the given shift factor, move the peaks and their ROI by the closest integer
 * amount corresponding to the change.
 * 
 * This function will automatically populate the FFT output array up to the Nyquist frequency
 * as well as a running phase compensation array
 * 
 * @param shift_factor - Factor by which to shift frequency data
 * @param gain - Gain applied to shifted peaks
 */
void shift_peaks(float shift_factor, float gain);

#endif // __PEAK_SHIFT__