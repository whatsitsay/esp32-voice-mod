#ifndef __FILTERS_H__
#define __FILTERS_H__

#define TRUE_ENV_NUM_ITERATIONS (3)   // Should be close enough for near convergence
#define CEPSTRUM_LEN            (N_SAMPLES / 2) // Half size for lower footprint and quicker calculations
#define CEPSTRUM_MOD_SIZE       (CEPSTRUM_LEN / 2 + 1) // Half again for magnitude

/**
 * @brief Configure true envelope buffers and sampling frequency
 * 
 * @param fft_buff - Pointer to buffer for FFT calculations
 * @param cepstrum_buff - Pointer to buffer to temporarily store cepstrum
 * @param sampling_freq_hz - Sampling frequency in Hz
 */
void config_true_env_calc(float* fft_buff, float* cepstrum_buff, float sampling_freq_hz);

/**
 * @brief Perform cepstral smoothing on spectrum
 * 
 * @param mag_log_ptr - Magnitude of FFT, represented in log
 * @param cepstrum_ptr - Cepstrum output buffer, should be size num_samples/2+1
 * @param cutoff_freq - Low-pass cutoff frequency for cepstrum smoothing
 */
void calc_cepstrum(float* mag_log_ptr, float* cepstrum_ptr, float cutoff_freq);

/**
 * @brief Calculate true envelope using a subsampled signal array
 * 
 * Will upsample and interpolate envelope to match FFT spectrum length
 * 
 * @param mag_log_ptr - Pointer to log magnitude array
 * @param env_ptr - Envelope pointer
 * @param cutoff_freq - Low-pass cutoff frequency for cepstrum smoothing
 */
void calc_true_envelope(float* mag_log_ptr, float* env_ptr, float cutoff_freq);

#endif // __FILTERS_H__