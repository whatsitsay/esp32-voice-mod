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
#define BAND_DIV_NBINS (8) // Number of bins for each 'band' when calculating peaks
#define FUNDAMENTAL_FREQ_MAX_BIN (150) // Maximum bin for estimating the fundamental frequency (empirical)

// Parameters for low-pitch index correction, as suggested by Roebel/Rodet
// I.e., bins closer to the perceived fundamental are affected less by the
// spectral envelope than higher bins
// Should prevent low-pitch shifting from becoming too dull
#define TRANSITION_BANDWIDTH (40) // Empirical
#define ENVELOPE_ASYMPTOTE_MAX (8) // 1/(1+exp(x)) is < 60 dB from asymptotes when |x|>7.6~=8
// For LUT, give just enough room to reach 60 dB point of asymptotes
#define IDX_CORR_SIZE ((2 * TRANSITION_BANDWIDTH * ENVELOPE_ASYMPTOTE_MAX) + 1)
#define IDX_CORR_FUNDAMENTAL (TRANSITION_BANDWIDTH * ENVELOPE_ASYMPTOTE_MAX)

// Hop phase correction for 50% overlap
// Derived from the formula (2pikH)/N, where H/N==0.5=> pi*k, for multiples of 2pi
#define HOP_PHASE_CORRECTION(k) ((k % 2) ? M_PI : 0)

typedef struct {
  float bin_freq_step;        // Frequency increment per bin of FFT array (sampling freq/N)
  float* fft_ptr;             // Pointer to input FFT array of current frame (size 2*N)
  float* fft_prev;            // Previous frame FFT
  float* fft_mag_db;          // Pointer to input FFT dB magnitude array of current frame (size N/2+1)
  float* fft_mag_raw;         // Pointer to input FFT magnitude array (raw values)
  float* fft_out_ptr;         // Pointer to output FFT (size 2*N)
  float* fft_out_prev_phase;  // Pointer to output FFT phase for the previous frame (size N/2 + 1)
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