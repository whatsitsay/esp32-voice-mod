/**
 * @file algo_common.c
 * @author Gabriel Kaufman (whatsitsay)
 * @brief Implementation for algo_common library for all phase vocoder techniques
 * @version 0.1
 * @date 2023-09-29
 * 
 */

#include <math.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_dsp.h"
#include "freertos/FreeRTOS.h"
#include "algo_common.h"

esp_err_t init_dsp_coeffs(int num_samples, float* hann_win)
{
  esp_err_t resp = ESP_OK;
  // Generate sin/cos coefficients for FFT calculations
  resp |= dsps_fft2r_init_fc32(NULL, num_samples);

  // Generate Hann window coefficients
  dsps_wind_hann_f32(hann_win, num_samples);

  return resp;
}

esp_err_t calc_fft(float* data_arr, int num_samples)
{
  esp_err_t resp = ESP_OK;
  // Calculate FFT itself from signal data
  resp |= dsps_fft2r_fc32(data_arr, num_samples); 
  // Perform bit reversal (necessary due to nature of implementation)
  resp |= dsps_bit_rev_fc32(data_arr, num_samples); 
  return resp;
}

void fill_mirror_fft(float* fft_arr, int num_samples)
{
  // Ensure size matches expected: 2 x num samples given real + imag
  configASSERT( sizeof(fft_arr) >= 2 * num_samples * sizeof(float) );

  for (int i = 0; i < num_samples; i+= 2)
  {
    // For real signals, second half after Nyquist freq will mirror
    // first half, but with complex conjugate.
    // Thus, copy first half of array in reverse order to the end
    // of the array and flip the sign of the imaginary component (2nd)
    fft_arr[2 * num_samples - 1 - i] = fft_arr[i];
    fft_arr[2 * num_samples - 2 - i] = fft_arr[i+1] * -1; // Sign flipped for conjugate
  }
}

esp_err_t inv_fft(float* fft_arr, int num_samples)
{
    // Perform FFT on conjugate of original FFT
    // Invert all imaginary values by multiplying by -1
    // This entails all odd entries
    for (int i = 0; i < num_samples; i++)
    {
        fft_arr[2 * i + 1] *= -1;
    }

    // Calc FFT from conjugate-mirrored data
    esp_err_t resp = calc_fft(fft_arr, num_samples);

    // Modify resultant iFFT array
    for (int i = 0; i < num_samples; i++)
    {
        // Correct all real values by sample size
        fft_arr[2 * i] /= (float)num_samples; // Correction factor
        // Conjugate of imaginary component (should be close to 0)
        fft_arr[2 * i + 1] /= (float)num_samples;
        fft_arr[2 * i + 1] *= -1;
    }

    return resp;
}

void calc_fft_mag(float* fft_arr, float* fft_mag, int num_samples)
{
  for (int i = 0; i < num_samples; i++) {
    float real = fft_arr[2 * i];
    float imag = fft_arr[2 * i + 1];

    fft_mag[i] = sqrtf(powf(real, 2) + powf(imag, 2));
  }
}

float get_idx_phase(float* fft_arr, int idx)
{
  // Assumes FFT is in the format arr[2 * N] with odd samples being real, and even imaginary
  float real = fft_arr[2 * idx];
  float imag = fft_arr[2 * idx + 1];

  // Ensure there is no divide-by-zero operation
  if (imag == 0) return 0.0; // Same as atan(0)
  if (real == 0) return M_PI / 2; // Exact result of atan(inf)
  return atanf(imag / real);
}

void calc_fft_phase(float* fft_arr, float* fft_phase, int num_samples)
{
  for (int i = 0; i < num_samples; i++) {
    fft_phase[i] = get_idx_phase(fft_arr, i);
  }
}

void mult_complex(float x_real, float x_imag, float y_real, float y_imag, float* prod_real, float* prod_imag)
{
  *prod_real = (x_real * y_real) - (x_imag * y_imag);
  *prod_imag = (x_real * y_imag) + (x_imag * y_real);
}

void polar_to_complex(float mag, float angle, float* cpx_real, float* cpx_imag)
{
  // Real is the cosine of the angle, imaginary the sine
  *cpx_real = mag * cosf(angle);
  *cpx_imag = mag * cosf(angle);
}