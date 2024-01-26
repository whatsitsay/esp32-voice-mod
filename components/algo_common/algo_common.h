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

#include <stdbool.h>
#include "esp_err.h"

#define MAX(x, y) ((x > y) ? x : y)
#define MIN(x, y) ((x < y) ? x : y)

#define N_SAMPLES (2048) // Number of samples for FFT operations
#define FFT_MOD_SIZE (N_SAMPLES/2 + 1) // Number of samples for modification, ie up to Nyquist
#define I2S_VOLTAGE_CONV (3.576279113e-7) // 3.0 V / max I2S (1 << 23 - 1)

// Useful for flag arrays
#define SET_ARR_BIT(arr, idx, val) (arr[(idx)/8] |= ((val) ? 1 : 0) << ((idx) % 8))
#define GET_ARR_BIT(arr, idx)      ((arr[(idx)/8] >> ((idx) % 8)) & 0x1)

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
 * @brief Calculate and store magnitudes for FFT array based on raw values
 * 
 * @param fft_arr - FFT array (input)
 * @param fft_mag - FFT magnitudes  (output)
 * @param num_samples - Number of samples in array
 * @return float - Maximum magnitude of samples
 */
float calc_fft_mag_raw(float* fft_arr, float* fft_mag, int num_samples);

/**
 * @brief Calculate and store magnitudes for FFT array in dB
 * 
 * @param fft_arr - FFT array (input)
 * @param fft_mag - FFT magnitudes in dB (output)
 * @param num_samples - Number of samples in array
 * @return float - Maximum magnitude of samples (in dB)
 */
float calc_fft_mag_db(float* fft_arr, float* fft_mag, int num_samples);

/**
 * @brief Helper function for converting raw value from I2S stream to dB based on voltage
 * 
 * @param raw_i2s - I2S raw value
 * @return float - dB based on voltage
 */
float conv_i2s_to_db(float raw_i2s);

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

/**
 * @brief Interpolate x between x_1 and x_0
 * 
 * Can also be used to extrapolate if x > x_1 (identical equation)
 * 
 * @param x - Desired interpolation point
 * @param x_1 - Upper bound of interpolation 
 * @param x_0 - Lower bound of interpolation
 * @param y_arr - Second coordinate for interpolation result
 * @return float - Interpolated/extrapolated Y value
 */
float interpolate_val(int x, int x_1, int x_0, float* y_arr);

#endif // __ALGO_COMMON_H__