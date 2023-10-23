
/**
 * @file filters.c
 * @author Gabe Kaufman (gkaufman93@gmail.com)
 * @brief Implementation for various frequency-domain filters for us in effects
 * @version 0.1
 * @date 2023-10-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "filters.h"
#include <algo_common.h>

static float euler_coeffs[N_SAMPLES * 2];

void init_euler_coeffs()
{
  // Only using delay taps, so use negative sine coefficients
  for (int k = 0; k < N_SAMPLES; k++)
  {
    euler_coeffs[2 * k]     = cosf((2 * M_PI * k) / N_SAMPLES); // Real component
    euler_coeffs[2 * k + 1] = sinf((-2 * M_PI * k) / N_SAMPLES); // Imag component
  }

}

float get_euler_coeff(int angle_idx, bool imag_comp)
{
  int corr_angle_idx = angle_idx % N_SAMPLES; // Correct for number of samples
  int array_idx = (imag_comp) ? 2 * corr_angle_idx + 1 : 2 * corr_angle_idx;

  return euler_coeffs[array_idx];
}

void allpass_filter(float ap_gain, int ap_angle_idx, float in_real, float in_imag, float* out_real, float* out_imag)
{
  float ap_real, ap_imag;
  // Transfer function for an all pass is given by the equation:
  // H(z) = (g + z^-N) / (1 + g * z^-N)
  // Numerator values
  float ap_num_real = ap_gain + get_euler_coeff(ap_angle_idx, false);
  float ap_num_imag = get_euler_coeff(ap_angle_idx, true);
  // Denominator values
  float ap_den_real = 1 + ap_gain * get_euler_coeff(ap_angle_idx, false);
  float ap_den_imag = ap_gain * get_euler_coeff(ap_angle_idx, true);

  // Calculate division
  divide_complex(ap_num_real, ap_num_imag, ap_den_real, ap_den_imag, &ap_real, &ap_imag);

  // Multiply with input and pass to output
  mult_complex(ap_real, ap_imag, in_real, in_imag, out_real, out_imag);
}

void comb_filter(float comb_gain, int comb_angle_idx, float in_real, float in_imag, float* out_real, float* out_imag)
{
    float comb_real, comb_imag;
    // Transfer function for simple comb filter is given by the equation:
    // H(z) = 1 / (1 - g * z^-N)
    // Denominator values
    float comb_den_real = 1 - comb_gain * get_euler_coeff(comb_angle_idx, false);
    float comb_den_imag = -1 * comb_gain * get_euler_coeff(comb_angle_idx, true);

    // Calculate division of 1
    divide_complex(1, 0, comb_den_real, comb_den_imag, &comb_real, &comb_imag);

  // Multiply by input, store in output
  mult_complex(comb_real, comb_imag, in_real, in_imag, out_real, out_imag);
}