
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
#include <algo_common.h>
#include "filters.h"

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

void init_reverb_coeffs(float* reverb_coeffs, int arr_length, int sampling_freq_hz)
{
    // Calculate all gains and store in temporary luts
    float comb_gain[] = {
        REVERB_FILTER_GAIN(REVERB_COMB1_TAU_MS, REVERB_COMB_RVT_MS),
        REVERB_FILTER_GAIN(REVERB_COMB2_TAU_MS, REVERB_COMB_RVT_MS),
        REVERB_FILTER_GAIN(REVERB_COMB3_TAU_MS, REVERB_COMB_RVT_MS),
        REVERB_FILTER_GAIN(REVERB_COMB4_TAU_MS, REVERB_COMB_RVT_MS)
    };
    float ap_gain[] = {
        REVERB_FILTER_GAIN(REVERB_AP1_TAU_MS, REVERB_AP1_RVT_MS),
        REVERB_FILTER_GAIN(REVERB_AP2_TAU_MS, REVERB_AP2_RVT_MS)
    };


    // Calculate all delay indicies, which will be delay (loop) time x sampling frequency
    int comb_delay_idx[] = {
        roundf(REVERB_COMB1_TAU_MS * sampling_freq_hz / 1000.0),
        roundf(REVERB_COMB2_TAU_MS * sampling_freq_hz / 1000.0),
        roundf(REVERB_COMB3_TAU_MS * sampling_freq_hz / 1000.0),
        roundf(REVERB_COMB4_TAU_MS * sampling_freq_hz / 1000.0)
    };
    int ap_delay_idx[] = {
        roundf(REVERB_AP1_TAU_MS * sampling_freq_hz / 1000.0),
        roundf(REVERB_AP2_TAU_MS * sampling_freq_hz / 1000.0)
    };

    // Loop over coefficient LUT
    for (int k = 0; k < arr_length; k++)
    {
        // Start with impulse 1 + 0j
        float ap_real = 1; 
        float ap_imag = 0;
        // Cascade multiply allpass coefficients
        for (int i = 0; i < REVERB_NUM_AP_FILTERS; i++) {
            allpass_filter(ap_gain[i], ap_delay_idx[i] * k, ap_real, ap_imag, &ap_real, &ap_imag);
        }

        // Calculate comb filters in parallel, so each gets an impulse response as input
        float comb_real = 0;
        float comb_imag = 0;
        for (int i = 0; i < REVERB_NUM_COMB_FILTERS; i++) {
            float curr_comb_real, curr_comb_imag;
            comb_filter(comb_gain[i], comb_delay_idx[i] * k, 1, 0, &curr_comb_real, &curr_comb_imag);
            // Comb filters are summed, so add to running sum here
            // Also correct for number of filters when adding
            comb_real += curr_comb_real / REVERB_NUM_COMB_FILTERS;
            comb_imag += curr_comb_imag / REVERB_NUM_COMB_FILTERS;
        }

        // Reverb coefficients will be product of AP and comb filters
        mult_complex(ap_real, ap_imag, comb_real, comb_imag, reverb_coeffs + 2*k, reverb_coeffs + 2*k + 1);
    }
}