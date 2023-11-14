#ifndef __ALGO_COMMON_H__
#define __ALGO_COMMON_H__
/**
 * @file algo_common.c
 * @author Gabriel Kaufman (whatsitsay)
 * @brief Header for algo_common library for all phase vocoder techniques
 * @version 0.1
 * @date 2023-09-29
 * 
 */

#include "esp_err.h"

#define MAX(x, y) ((x > y) ? x : y)
#define MIN(x, y) ((x < y) ? x : y)
#define FFT_DB_BASE (1e4) // Base value for comparison when calculating dB values of magnitude

#define N_SAMPLES (4096) // Number of samples for FFT operations
#define FFT_MOD_SIZE ((N_SAMPLES / 2) + 1) // Number of samples needed to properly modify real-signal FFTs

/**
 * @brief Calculate FFT from signal data
 * 
 * @param data_arr - signal data, in the format [Real, Imag, Real, Imag,...]
 * @param num_samples - Number of samples in data array/resultant FFT
 * @return esp_err_t - OR'ed result of operations (ESP_OK if successful)
 */
esp_err_t calc_fft(float* data_arr, int num_samples);

/**
 * @brief Fill real-data FFT with conjugate mirror
 * 
 * For real signals, the FFT will always be symmetric about the Nyquist freq
 * (midpoint), where it will mirror the former half but as their complex conjugates.
 * I.e., with each imaginary component sign flipped
 * 
 * As such, this function fills the latter half of an FFT array with this mirrored data.
 * 
 * @param fft_arr - FFT array, at least half-filled
 * @param num_samples - Full size of FFT (*not* half)
 */
void fill_mirror_fft(float* fft_arr, int num_samples);

/**
 * @brief Inverse FFT
 * 
 * Stand-in for iFFT algorithm, using forward FFT implementation.
 * Assumes FFT is of real signal.
 * 
 * @param fft_arr - FFT array
 * @param num_samples - Number of samples in array
 * @return esp_err_t - ESP_OK, Error otherwise
 */
esp_err_t inv_fft(float* fft_arr, int num_samples);

/**
 * @brief Calculate and store magnitudes for FFT array in dB
 * 
 * Compared to 10^4 (TODO: make param?)
 * 
 * @param fft_arr - FFT array (input)
 * @param fft_mag - FFT magnitudes in dB (output)
 * @param num_samples - Number of samples in array
 */
void calc_fft_mag_db(float* fft_arr, float* fft_mag, int num_samples);

/**
 * @brief Calculate the FFT phase at the given index
 * 
 * Assumes the FFT array referenced by the pointer is populated, with
 * the index being for the true array (i.e. of N samples).
 * 
 * @param fft_arr - Populated FFT array, with the format real, imag alternating
 * @param idx - FFT index (NOT corrected for format)
 * @return float - Phase in radians, calculated as atanf(imag / real)
 */
float get_idx_phase(float* fft_arr, int idx);

/**
 * @brief Calculate and store phases for FFT array in radians
 * 
 * Values should be in the set of [-PI, PI]
 * 
 * @param fft_arr - FFT array (input)
 * @param fft_phase - FFT phases (output)
 * @param num_samples - Number of samples in array
 */
void calc_fft_phase(float* fft_arr, float* fft_phase, int num_samples);


/**
 * @brief Helper function for multiplying two complex numbers and storing the result into pointers
 * 
 * @param x_real - First complex number, real component
 * @param x_imag - First complex number, imaginary component
 * @param y_real - Second complex number, real component
 * @param y_imag - Second complex number, imaginary component
 * @param prod_real - Product real component (output)
 * @param prod_imag - Product imaginary component (output)
 */
void mult_complex(float x_real, float x_imag, float y_real, float y_imag, float* prod_real, float* prod_imag);

/**
 * @brief Helper function for dividing two complex numbers and storing the result into pointers
 * 
 * @param x_real - First complex number, real component
 * @param x_imag - First complex number, imaginary component
 * @param y_real - Second complex number, real component
 * @param y_imag - Second complex number, imaginary component
 * @param div_real - Division real component (output)
 * @param div_imag - Division imaginary component (output)
 */
void divide_complex(float x_real, float x_imag, float y_real, float y_imag, float* div_real, float* div_imag);

/**
 * @brief Convert polar coordinates (magnitude/angle) into complex cartesian (real/imagary)
 * 
 * @param mag - Magnitude/radius
 * @param angle - Angle
 * @param cpx_real - Real component (output)
 * @param cpx_imag - Imaginary component (output)
 */
void polar_to_complex(float mag, float angle, float* cpx_real, float* cpx_imag);

/**
 * @brief Get the euler coefficient for the given discrete angle
 * 
 * Automatically corrected for by N_SAMPLES
 * 
 * @param angle_idx - Index of discrete angle
 * @param imag_comp - True if imaginary component, false otherwise
 * @return float - Coefficient value
 */
float get_euler_coeff(int angle_idx, bool imag_comp);

/**
 * @brief Get window coefficient at the given index
 * 
 * Currently implemented to use root-Hann window for proper reconstruction
 * 
 * Defined for N=4096
 * 
 * @param idx - Current window index
 * @return float - Value of window at index
 */
float get_window(int idx);

#endif // __ALGO_COMMON_H__