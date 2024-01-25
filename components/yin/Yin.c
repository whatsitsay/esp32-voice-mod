/**
 * @file Yin.c
 * @author  ashokfernandez (edited by Gabe Kaufman slightly)
 * @brief Implementation for Yin algorithm helper. Based almost entirely on https://github.com/ashokfernandez/Yin-Pitch-Tracking,
 * with some small tweaks (int32 instead of int16, sample rate define later)
 * @version 0.1
 * @date 2024-01-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <stdint.h> /* For standard interger types (int) */
#include <stdlib.h> /* For call to malloc */
#include "Yin.h"
#include "algo_common.h"

/* ------------------------------------------------------------------------------------------
--------------------------------------------------------------------------- PRIVATE FUNCTIONS
-------------------------------------------------------------------------------------------*/

/**
 * Step 1: Calculates the squared difference of the signal with a shifted version of itself.
 * @param buffer Buffer of samples to process. 
 *
 * This is the Yin algorithms tweak on autocorellation. Read http://audition.ens.fr/adc/pdf/2002_JASA_YIN.pdf
 * for more details on what is in here and why it's done this way.
 */
void Yin_difference(Yin *yin, int* buffer){
	/* Calculate the difference for difference shift values (tau) for the half of the samples */
	for(uint16_t tau = 0 ; tau < yin->halfBufferSize; tau++){
    unsigned total = 0;
		/* Take the difference of the signal with a shifted version of itself, then square it.
		 * (This is the Yin algorithm's tweak on autocorellation) */ 
		for(uint16_t i = 0; i < yin->halfBufferSize; i += 4){
			int delta_int = buffer[i] - buffer[i + tau];
      // Store in int16 to save space, bitshifting difference down with sign extension
      int16_t delta = (delta_int >> 16) | ((delta_int < 0) ? 0xFFFF8000 : 0x0);
      // Get square
			total += delta * delta;
		}
    yin->yinBuffer[tau] = total; // Convert to float only at the end to save time
	}
}


/**
 * Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1
 * @param yin #Yin structure with information about the signal
 *
 * This goes through the Yin autocorellation values and finds out roughly where shift is which 
 * produced the smallest difference
 */
void Yin_cumulativeMeanNormalizedDifference(Yin *yin){
	int tau;
	float runningSum = 0;
	yin->yinBuffer[0] = 1;

	/* Sum all the values in the autocorellation buffer and nomalise the result, replacing
	 * the value in the autocorellation buffer with a cumulative mean of the normalised difference */
	for (tau = 1; tau < yin->halfBufferSize; tau++) {
		runningSum += yin->yinBuffer[tau];
		yin->yinBuffer[tau] *= tau / runningSum;
	}
}

/**
 * Step 3: Search through the normalised cumulative mean array and find values that are over the threshold
 * @return Shift (tau) which caused the best approximate autocorellation. -1 if no suitable value is found over the threshold.
 */
int Yin_absoluteThreshold(Yin *yin){
	int tau;

	/* Search through the array of cumulative mean values, and look for ones that are over the threshold 
	 * The first two positions in yinBuffer are always so start at the third (index 2) */
	for (tau = 2; tau < yin->halfBufferSize ; tau++) {
		if (yin->yinBuffer[tau] < yin->threshold) {
			while (tau + 1 < yin->halfBufferSize && yin->yinBuffer[tau + 1] < yin->yinBuffer[tau]) {
				tau++;
			}
			/* found tau, exit loop and return
			 * store the probability
			 * From the YIN paper: The yin->threshold determines the list of
			 * candidates admitted to the set, and can be interpreted as the
			 * proportion of aperiodic power tolerated
			 * within a periodic signal.
			 *
			 * Since we want the periodicity and and not aperiodicity:
			 * periodicity = 1 - aperiodicity */
			yin->probability = 1 - yin->yinBuffer[tau];
			break;
		}
	}

	/* if no pitch found, tau => -1 */
	if (tau == yin->halfBufferSize || yin->yinBuffer[tau] >= yin->threshold) {
		tau = -1;
		yin->probability = 0;
	}

	return tau;
}

/**
 * Step 5: Interpolate the shift value (tau) to improve the pitch estimate.
 * @param  yin         [description]
 * @param  tauEstimate [description]
 * @return             [description]
 *
 * The 'best' shift value for autocorellation is most likely not an interger shift of the signal.
 * As we only autocorellated using integer shifts we should check that there isn't a better fractional 
 * shift value.
 */
float Yin_parabolicInterpolation(Yin *yin, int tauEstimate) {
	float betterTau;
	int x0;
	int x2;
	
	/* Calculate the first polynomial coeffcient based on the current estimate of tau */
	if (tauEstimate < 1) {
		x0 = tauEstimate;
	} 
	else {
		x0 = tauEstimate - 1;
	}

	/* Calculate the second polynomial coeffcient based on the current estimate of tau */
	if (tauEstimate + 1 < yin->halfBufferSize) {
		x2 = tauEstimate + 1;
	} 
	else {
		x2 = tauEstimate;
	}

	/* Algorithm to parabolically interpolate the shift value tau to find a better estimate */
	if (x0 == tauEstimate) {
		if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x2]) {
			betterTau = tauEstimate;
		} 
		else {
			betterTau = x2;
		}
	} 
	else if (x2 == tauEstimate) {
		if (yin->yinBuffer[tauEstimate] <= yin->yinBuffer[x0]) {
			betterTau = tauEstimate;
		} 
		else {
			betterTau = x0;
		}
	} 
	else {
		float s0, s1, s2;
		s0 = yin->yinBuffer[x0];
		s1 = yin->yinBuffer[tauEstimate];
		s2 = yin->yinBuffer[x2];
		// fixed AUBIO implementation, thanks to Karl Helgason:
		// (2.0f * s1 - s2 - s0) was incorrectly multiplied with -1
		betterTau = tauEstimate + (s2 - s0) / (2 * (2 * s1 - s2 - s0));
	}


	return betterTau;
}





/* ------------------------------------------------------------------------------------------
---------------------------------------------------------------------------- PUBLIC FUNCTIONS
-------------------------------------------------------------------------------------------*/



/**
 * Initialise the Yin pitch detection object
 * @param yin        Yin pitch detection object to initialise
 * @param bufferSize Length of the audio buffer to analyse
 * @param threshold  Allowed uncertainty (e.g 0.05 will return a pitch with ~95% probability)
 */
void Yin_init(Yin *yin, int bufferSize, float threshold, float* yinBuffPtr){
	/* Initialise the fields of the Yin structure passed in */
	yin->bufferSize = bufferSize;
	yin->halfBufferSize = bufferSize / 2;
	yin->probability = 0.0;
	yin->threshold = threshold;

	/* Allocate the autocorellation buffer and initialise it to zero , if needed*/
  if (yinBuffPtr == NULL) {
    yin->yinBuffer = (float *) calloc(yin->halfBufferSize, sizeof(float));
  } else {
    // Use given buffer
    yin->yinBuffer = yinBuffPtr;
  }
}

/**
 * Runs the Yin pitch detection algortihm
 * @param  yin    Initialised Yin object
 * @param  buffer Buffer of samples to analyse
 * @return        Fundamental frequency of the signal in Hz. Returns -1 if pitch can't be found
 */
float Yin_getPitch(Yin *yin, int* buffer, float sampling_rate_hz){
	int tauEstimate = -1;
	float pitchInHertz = -1;
	
	/* Step 1: Calculates the squared difference of the signal with a shifted version of itself. */
	Yin_difference(yin, buffer);
	
	/* Step 2: Calculate the cumulative mean on the normalised difference calculated in step 1 */
	Yin_cumulativeMeanNormalizedDifference(yin);
	
	/* Step 3: Search through the normalised cumulative mean array and find values that are over the threshold */
	tauEstimate = Yin_absoluteThreshold(yin);
	
	/* Step 5: Interpolate the shift value (tau) to improve the pitch estimate. */
	if(tauEstimate != -1){
		pitchInHertz = sampling_rate_hz / Yin_parabolicInterpolation(yin, tauEstimate);
	}
	
	return pitchInHertz;
}

/**
 * Certainty of the pitch found 
 * @param  yin Yin object that has been run over a buffer
 * @return     Returns the certainty of the note found as a decimal (i.e 0.3 is 30%)
 */
float Yin_getProbability(Yin *yin){
	return yin->probability;
}
