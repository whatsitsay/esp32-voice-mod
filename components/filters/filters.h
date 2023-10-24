#ifndef __FILTERS_H__
#define __FILTERS_H__

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

#endif // __FILTERS_H__