#ifndef __FILTERS_H__
#define __FILTERS_H__

// Schroeder Reverb filter parameters
#define REVERB_NUM_COMB_FILTERS            (4) // In parallel
#define REVERB_NUM_AP_FILTERS              (2) // In series
#define REVERB_COMB_RVT_MS                 (1000.0) // "Reverb time desired", time it takes delayed signal to reach -60 dB
#define REVERB_AP1_RVT_MS                  (96.83)
#define REVERB_AP2_RVT_MS                  (32.92)
#define REVERB_COMB1_TAU_MS                (29.7) // Loop time of comb filter 1
#define REVERB_COMB2_TAU_MS                (37.1) // Loop time of comb filter 2
#define REVERB_COMB3_TAU_MS                (41.1) // Loop time of comb filter 3
#define REVERB_COMB4_TAU_MS                (43.7) // Loop time of comb filter 4
#define REVERB_AP1_TAU_MS                  (5.0)  // Loop time of allpass filter 1
#define REVERB_AP2_TAU_MS                  (1.7)  // Loop time of allpass filter 2
#define REVERB_FILTER_GAIN(tau_ms, rvt_ms) (powf(0.001, tau_ms / rvt_ms)) // Gain based on loop time

/**
 * @brief Allpass filter
 * 
 * Frequency-domain all-pass filter defined by the transfer function:
 * H(z) = (g + z^-N) / (1 + g * z^-N)
 * 
 * @param ap_gain - Gain of both feedforward and feedback paths (g)
 * @param ap_angle_idx - Current angle index of fourier transform
 * @param in_real - Input data real component X(z)
 * @param in_imag - Input data imag component X(z)
 * @param out_real - Output data real component Y(z) = H(z)X(z)
 * @param out_imag - Output data imag component Y(z) = H(z)X(z)
 */
void allpass_filter(float ap_gain, int ap_angle_idx, float in_real, float in_imag, float* out_real, float* out_imag);

/**
 * @brief Comb filter
 * 
 * Frequency-domain comb filter defined by the transfer function:
 * H(z) = 1 / (1 - g * z^-N)
 * 
 * @param comb_gain - Gain of feedback path (g)
 * @param comb_angle_idx - Current angle index of fourier transform
 * @param in_real - Input data real component X(z)
 * @param in_imag - Input data imag component X(z)
 * @param out_real - Output data real component Y(z) = H(z)X(z)
 * @param out_imag - Output data imag component Y(z) = H(z)X(z)
 */
void comb_filter(float comb_gain, int comb_angle_idx, float in_real, float in_imag, float* out_real, float* out_imag);

/**
 * @brief Initialize Schroeder reverberation coefficient array
 * 
 * Fill array of coefficients for Schroeder reverberation filter in the
 * frequency domain. It is implemented as two all-pass filters in series
 * followed by four comb filters in parallel.
 * 
 * Timing and gain parameters based on flyingSand reverberation constants
 * for a concert hall from their post "Digital Reverberation", which are themselves
 * derived from Dodge & Jerse, "Computer Music" (pg. 301)
 * (URL: https://christianfloisand.wordpress.com/2012/09/04/digital-reverberation/) 
 * 
 * @param reverb_coeffs - Table for reverberation coefficients
 * @param arr_length - Length of coefficient array
 * @param sampling_freq_hz - Audio data sampling frequency
 */
void init_reverb_coeffs(float* reverb_coeffs, int arr_length, int sampling_freq_hz);

#endif // __FILTERS_H__