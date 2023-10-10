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

#define PEAK_THRESHOLD (10) // Empirical TODO double-check

typedef struct {
  int num_samples;            // Number of samples N per FFT (*full* FFT, not just relevant portion)
  int hop_size;               // Hop size of analysis
  float bin_freq_step;        // Frequency increment per bin of FFT array (sampling freq/N)
  float* fft_ptr;             // Pointer to input FFT array of current frame (size 2*N)
  float* fft_prev_ptr;        // Pointer to input FFT array of previous frame (size 2*N)
  float* fft_mag_ptr;         // Pointer to input FFT magnitude array of current frame (size N)
  float* fft_out_ptr;         // Pointer to output FFT (size 2*N)
} peak_shift_cfg_t;

typedef struct {
  int idx;              // Index of peak in FFT array (non-doubled)
  int left_bound;       // Leftmost index of ROI (non-doubled)
  int right_bound;      // Rightmost index of ROI (non-doubled)
  float inst_freq;      // Instantaneous frequency represented by peak
  float phase;          // Phase value of peak
} peak_data_t;

#define MAX_PEAKS (100) // Somewhat of a fudge factor to limit memory footprint

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
 * @brief Reset cumulative phase compensation array
 * 
 * Sets all complex values to 1 + 0j
 * Needs to be a pointer in the case of chorus effect (multiple pitch shifts)
 * 
 * @param run_phase_comp_ptr - Pointer to running phase compensation array.
 *                             Should be cfg->num_samples in length
 */
void reset_phase_comp_arr(float* run_phase_comp_ptr);

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
 * @brief Shift peaks by integer number of frequency bins
 * 
 * Based on the given shift factor, move the peaks and their ROI by the closest integer
 * amount corresponding to the change.
 * 
 * This function will automatically populate the FFT output array up to the Nyquist frequency
 * as well as a running phase compensation array
 * 
 * @param shift_factor - Factor by which to shift frequency data
 */
void shift_peaks_int(float shift_factor, float* run_phase_comp_ptr);