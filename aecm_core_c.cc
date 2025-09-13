/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stddef.h>
#include <stdlib.h>

#include "aecm_core.h"

extern "C" {
#include "ring_buffer.h"
#include "real_fft.h"
}
#include "echo_control_mobile.h"
#include "delay_estimator_wrapper.h"

 
 

// Square root of Hanning window in Q14.
static const ALIGN8_BEG int16_t Aecm_kSqrtHanning[] ALIGN8_END = {
    0,     399,   798,   1196,  1594,  1990,  2386,  2780,  3172,  3562,  3951,
    4337,  4720,  5101,  5478,  5853,  6224,  6591,  6954,  7313,  7668,  8019,
    8364,  8705,  9040,  9370,  9695,  10013, 10326, 10633, 10933, 11227, 11514,
    11795, 12068, 12335, 12594, 12845, 13089, 13325, 13553, 13773, 13985, 14189,
    14384, 14571, 14749, 14918, 15079, 15231, 15373, 15506, 15631, 15746, 15851,
    15947, 16034, 16111, 16179, 16237, 16286, 16325, 16354, 16373, 16384};

// 近似版の振幅計算は使わず、sqrtベースのみを使用

// CNG (Comfort Noise) は教育用最小構成から削除

static void WindowAndFFT(AecmCore* aecm,
                         int16_t* fft,
                         const int16_t* time_signal,
                         ComplexInt16* freq_signal,
                         int time_signal_scaling) {
  int i = 0;

  // FFT of signal
  for (i = 0; i < PART_LEN; i++) {
    // Window time domain signal and insert into real part of
    // transformation array `fft`
    int16_t scaled_time_signal = time_signal[i] * (1 << time_signal_scaling);
    fft[i] = (int16_t)((scaled_time_signal * Aecm_kSqrtHanning[i]) >> 14);
    scaled_time_signal = time_signal[i + PART_LEN] * (1 << time_signal_scaling);
    fft[PART_LEN + i] = (int16_t)(
        (scaled_time_signal * Aecm_kSqrtHanning[PART_LEN - i]) >> 14);
  }

  // Do forward FFT, then take only the first PART_LEN complex samples,
  // and change signs of the imaginary parts.
  RealForwardFFT(aecm->real_fft, fft, (int16_t*)freq_signal);
  for (i = 0; i < PART_LEN; i++) {
    freq_signal[i].imag = -freq_signal[i].imag;
  }
}

static void InverseFFTAndWindow(AecmCore* aecm,
                                int16_t* fft,
                                ComplexInt16* efw,
                                int16_t* output) {
  int i, j, outCFFT;
  int32_t tmp32no1;
  // Reuse `efw` for the inverse FFT output after transferring
  // the contents to `fft`.
  int16_t* ifft_out = (int16_t*)efw;

  // Synthesis
  for (i = 1, j = 2; i < PART_LEN; i += 1, j += 2) {
    fft[j] = efw[i].real;
    fft[j + 1] = -efw[i].imag;
  }
  fft[0] = efw[0].real;
  fft[1] = -efw[0].imag;

  fft[PART_LEN2] = efw[PART_LEN].real;
  fft[PART_LEN2 + 1] = -efw[PART_LEN].imag;

  // Inverse FFT. Keep outCFFT to scale the samples in the next block.
  outCFFT = RealInverseFFT(aecm->real_fft, fft, ifft_out);
  for (i = 0; i < PART_LEN; i++) {
    ifft_out[i] = (int16_t)MUL_16_16_RSFT_WITH_ROUND(
        ifft_out[i], Aecm_kSqrtHanning[i], 14);
    tmp32no1 = SHIFT_W32((int32_t)ifft_out[i],
                                    outCFFT - aecm->dfaCleanQDomain);
    output[i] = (int16_t)SAT(WORD16_MAX,
                                        tmp32no1 + aecm->outBuf[i],
                                        WORD16_MIN);

    tmp32no1 =
        (ifft_out[PART_LEN + i] * Aecm_kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = SHIFT_W32(tmp32no1, outCFFT - aecm->dfaCleanQDomain);
    aecm->outBuf[i] = (int16_t)SAT(WORD16_MAX, tmp32no1,
                                              WORD16_MIN);
  }

  // Copy the current block to the old position
  // (aecm->outBuf is shifted elsewhere)
  memcpy(aecm->xBuf, aecm->xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(aecm->dBufNoisy, aecm->dBufNoisy + PART_LEN,
         sizeof(int16_t) * PART_LEN);
}

// Transforms a time domain signal into the frequency domain, outputting the
// complex valued signal, absolute value and sum of absolute values.
//
// time_signal          [in]    Pointer to time domain signal
// freq_signal_real     [out]   Pointer to real part of frequency domain array
// freq_signal_imag     [out]   Pointer to imaginary part of frequency domain
//                              array
// freq_signal_abs      [out]   Pointer to absolute value of frequency domain
//                              array
// freq_signal_sum_abs  [out]   Pointer to the sum of all absolute values in
//                              the frequency domain array
// return value                 The Q-domain of current frequency values
//
static int TimeToFrequencyDomain(AecmCore* aecm,
                                 const int16_t* time_signal,
                                 ComplexInt16* freq_signal,
                                 uint16_t* freq_signal_abs,
                                 uint32_t* freq_signal_sum_abs) {
  int i = 0;
  int time_signal_scaling = 0;

  int32_t tmp32no1 = 0;
  int32_t tmp32no2 = 0;

  int16_t fft[PART_LEN4];

  int16_t tmp16no1;
  int16_t tmp16no2;

#ifdef AECM_DYNAMIC_Q
  tmp16no1 = MaxAbsValueW16(time_signal, PART_LEN2);
  time_signal_scaling = NormW16(tmp16no1);
#endif

  WindowAndFFT(aecm, fft, time_signal, freq_signal, time_signal_scaling);

  // Extract imaginary and real part, calculate the magnitude for
  // all frequency bins
  freq_signal[0].imag = 0;
  freq_signal[PART_LEN].imag = 0;
  freq_signal_abs[0] = (uint16_t)ABS_W16(freq_signal[0].real);
  freq_signal_abs[PART_LEN] =
      (uint16_t)ABS_W16(freq_signal[PART_LEN].real);
  (*freq_signal_sum_abs) =
      (uint32_t)(freq_signal_abs[0]) + (uint32_t)(freq_signal_abs[PART_LEN]);

  for (i = 1; i < PART_LEN; i++) {
    if (freq_signal[i].real == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].imag);
    } else if (freq_signal[i].imag == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].real);
    } else {
      tmp16no1 = ABS_W16(freq_signal[i].real);
      tmp16no2 = ABS_W16(freq_signal[i].imag);
      tmp32no1 = tmp16no1 * tmp16no1;
      tmp32no2 = tmp16no2 * tmp16no2;
      tmp32no2 = AddSatW32(tmp32no1, tmp32no2);
      tmp32no1 = SqrtFloor(tmp32no2);

      freq_signal_abs[i] = (uint16_t)tmp32no1;
    }
    (*freq_signal_sum_abs) += (uint32_t)freq_signal_abs[i];
  }

  return time_signal_scaling;
}

 

int Aecm_ProcessBlock(AecmCore* aecm,
                            const int16_t* farend,
                            const int16_t* nearend,
                            int16_t* output) {
  int i;

  uint32_t xfaSum;
  uint32_t dfaNoisySum;
  uint32_t echoEst32Gained;
  uint32_t tmpU32;

  int32_t tmp32no1;

  uint16_t xfa[PART_LEN1];
  uint16_t dfaNoisy[PART_LEN1];
  uint16_t* ptrDfaClean = dfaNoisy;
  const uint16_t* far_spectrum_ptr = NULL;

  // シンプルなローカル配列（手動アラインメント不要）
  int16_t fft[PART_LEN4 + 2];  // +2 to make a loop safe.
  int32_t echoEst32[PART_LEN1];
  ComplexInt16 dfw[PART_LEN2];
  ComplexInt16 efw[PART_LEN2];

  int16_t hnl[PART_LEN1];
  int16_t numPosCoef = 0;
  int16_t nlpGain = ONE_Q14;
  int delay;
  int16_t tmp16no1;
  int16_t tmp16no2;
  int16_t mu;
  int16_t supGain;
  int16_t zeros32, zeros16;
  int16_t zerosDBufNoisy, zerosXBuf;
  int far_q;
  int16_t resolutionDiff, qDomainDiff, dfa_clean_q_domain_diff;

  const int kMinPrefBand = 4;
  const int kMaxPrefBand = 24;
  int32_t avgHnl32 = 0;

  // Determine startup state. There are three states:
  // (0) the first CONV_LEN blocks
  // (1) another CONV_LEN blocks
  // (2) the rest

  if (aecm->startupState < 2) {
    aecm->startupState =
        (aecm->totCount >= CONV_LEN) + (aecm->totCount >= CONV_LEN2);
  }
  // END: Determine startup state

  // Buffer near and far end signals
  memcpy(aecm->xBuf + PART_LEN, farend, sizeof(int16_t) * PART_LEN);
  memcpy(aecm->dBufNoisy + PART_LEN, nearend, sizeof(int16_t) * PART_LEN);

  // Transform far end signal from time domain to frequency domain.
  far_q = TimeToFrequencyDomain(aecm, aecm->xBuf, dfw, xfa, &xfaSum);

  // Transform noisy near end signal from time domain to frequency domain.
  zerosDBufNoisy =
      TimeToFrequencyDomain(aecm, aecm->dBufNoisy, dfw, dfaNoisy, &dfaNoisySum);
  aecm->dfaNoisyQDomainOld = aecm->dfaNoisyQDomain;
  aecm->dfaNoisyQDomain = (int16_t)zerosDBufNoisy;

  // 近端クリーン経路は削除。クリーンQドメインはノイジーと等価に扱う。
  aecm->dfaCleanQDomainOld = aecm->dfaNoisyQDomainOld;
  aecm->dfaCleanQDomain = aecm->dfaNoisyQDomain;

  // Get the delay
  // Save far-end history and estimate delay
  Aecm_UpdateFarHistory(aecm, xfa, far_q);
  if (AddFarSpectrumFix(aecm->delay_estimator_farend, xfa,
                               far_q) == -1) {
    return -1;
  }
  delay = DelayEstimatorProcessFix(aecm->delay_estimator, dfaNoisy,
                                          zerosDBufNoisy);
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    // If the delay is unknown, we assume zero.
    // NOTE: this will have to be adjusted if we ever add lookahead.
    delay = 0;
  }

  if (aecm->fixedDelay >= 0) {
    // Use fixed delay
    delay = aecm->fixedDelay;
  }

  // Get aligned far end spectrum
  far_spectrum_ptr = Aecm_AlignedFarend(aecm, &far_q, delay);
  zerosXBuf = (int16_t)far_q;
  if (far_spectrum_ptr == NULL) {
    return -1;
  }

  // Calculate log(energy) and update energy threshold levels
  Aecm_CalcEnergies(aecm, far_spectrum_ptr, zerosXBuf, dfaNoisySum,
                          echoEst32);

  // Calculate stepsize
  mu = Aecm_CalcStepSize(aecm);

  // Update counters
  aecm->totCount++;

  // This is the channel estimation algorithm.
  // It is base on NLMS but has a variable step length,
  // which was calculated above.
  Aecm_UpdateChannel(aecm, far_spectrum_ptr, zerosXBuf, dfaNoisy, mu,
                           echoEst32);
  supGain = Aecm_CalcSuppressionGain(aecm);

  // Calculate Wiener filter hnl[]
  for (i = 0; i < PART_LEN1; i++) {
    // Far end signal through channel estimate in Q8
    // How much can we shift right to preserve resolution
    tmp32no1 = echoEst32[i] - aecm->echoFilt[i];
    aecm->echoFilt[i] += (int32_t)(((int64_t)tmp32no1 * 50) >> 8);

    zeros32 = NormW32(aecm->echoFilt[i]) + 1;
    zeros16 = NormW16(supGain) + 1;
    if (zeros32 + zeros16 > 16) {
      // Multiplication is safe
      // Result in
      // Q(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN+
      //   aecm->xfaQDomainBuf[diff])
      echoEst32Gained =
          UMUL_32_16((uint32_t)aecm->echoFilt[i], (uint16_t)supGain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      resolutionDiff += (aecm->dfaCleanQDomain - zerosXBuf);
    } else {
      tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff =
          14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      resolutionDiff += (aecm->dfaCleanQDomain - zerosXBuf);
      if (zeros32 > tmp16no1) {
        echoEst32Gained = UMUL_32_16((uint32_t)aecm->echoFilt[i],
                                                supGain >> tmp16no1);
      } else {
        // Result in Q-(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN-16)
        echoEst32Gained = (aecm->echoFilt[i] >> tmp16no1) * supGain;
      }
    }

    zeros16 = NormW16(aecm->nearFilt[i]);
    dfa_clean_q_domain_diff = aecm->dfaCleanQDomain - aecm->dfaCleanQDomainOld;
    if (zeros16 < dfa_clean_q_domain_diff && aecm->nearFilt[i]) {
      tmp16no1 = aecm->nearFilt[i] * (1 << zeros16);
      qDomainDiff = zeros16 - dfa_clean_q_domain_diff;
      tmp16no2 = ptrDfaClean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = dfa_clean_q_domain_diff < 0
                     ? aecm->nearFilt[i] >> -dfa_clean_q_domain_diff
                     : aecm->nearFilt[i] * (1 << dfa_clean_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = ptrDfaClean[i];
    }
    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
    tmp16no2 += tmp16no1;
    zeros16 = NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      aecm->nearFilt[i] = WORD16_MAX;
    } else {
      aecm->nearFilt[i] = qDomainDiff < 0 ? tmp16no2 * (1 << -qDomainDiff)
                                          : tmp16no2 >> qDomainDiff;
    }

    // Wiener filter coefficients, resulting hnl in Q14
    if (echoEst32Gained == 0) {
      hnl[i] = ONE_Q14;
    } else if (aecm->nearFilt[i] == 0) {
      hnl[i] = 0;
    } else {
      // Multiply the suppression gain
      // Rounding
      echoEst32Gained += (uint32_t)(aecm->nearFilt[i] >> 1);
      tmpU32 =
          DivU32U16(echoEst32Gained, (uint16_t)aecm->nearFilt[i]);

      // Current resolution is
      // Q-(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN- max(0,17-zeros16- zeros32))
      // Make sure we are in Q14
      tmp32no1 = (int32_t)SHIFT_W32(tmpU32, resolutionDiff);
      if (tmp32no1 > ONE_Q14) {
        hnl[i] = 0;
      } else if (tmp32no1 < 0) {
        hnl[i] = ONE_Q14;
      } else {
        // 1-echoEst/dfa
        hnl[i] = ONE_Q14 - (int16_t)tmp32no1;
        if (hnl[i] < 0) {
          hnl[i] = 0;
        }
      }
    }
    if (hnl[i]) {
      numPosCoef++;
    }
  }
  // Only in wideband. Prevent the gain in upper band from being larger than
  // in lower band.
  if (aecm->mult == 2) {
    // TODO(bjornv): Investigate if the scaling of hnl[i] below can cause
    //               speech distortion in double-talk.
    for (i = 0; i < PART_LEN1; i++) {
      hnl[i] = (int16_t)((hnl[i] * hnl[i]) >> 14);
    }

    for (i = kMinPrefBand; i <= kMaxPrefBand; i++) {
      avgHnl32 += (int32_t)hnl[i];
    }
    avgHnl32 /= (kMaxPrefBand - kMinPrefBand + 1);

    for (i = kMaxPrefBand; i < PART_LEN1; i++) {
      if (hnl[i] > (int16_t)avgHnl32) {
        hnl[i] = (int16_t)avgHnl32;
      }
    }
  }

  // Calculate NLP gain, result is in Q14
  if (aecm->nlpFlag) {
    for (i = 0; i < PART_LEN1; i++) {
      // Truncate values close to zero and one.
      if (hnl[i] > NLP_COMP_HIGH) {
        hnl[i] = ONE_Q14;
      } else if (hnl[i] < NLP_COMP_LOW) {
        hnl[i] = 0;
      }

      // Remove outliers
      if (numPosCoef < 3) {
        nlpGain = 0;
      } else {
        nlpGain = ONE_Q14;
      }

      // NLP
      if ((hnl[i] == ONE_Q14) && (nlpGain == ONE_Q14)) {
        hnl[i] = ONE_Q14;
      } else {
        hnl[i] = (int16_t)((hnl[i] * nlpGain) >> 14);
      }

      // multiply with Wiener coefficients
      efw[i].real = (int16_t)(
          MUL_16_16_RSFT_WITH_ROUND(dfw[i].real, hnl[i], 14));
      efw[i].imag = (int16_t)(
          MUL_16_16_RSFT_WITH_ROUND(dfw[i].imag, hnl[i], 14));
    }
  } else {
    // multiply with Wiener coefficients
    for (i = 0; i < PART_LEN1; i++) {
      efw[i].real = (int16_t)(
          MUL_16_16_RSFT_WITH_ROUND(dfw[i].real, hnl[i], 14));
      efw[i].imag = (int16_t)(
          MUL_16_16_RSFT_WITH_ROUND(dfw[i].imag, hnl[i], 14));
    }
  }

  // CNG 削除につき快適雑音の付加は行わない

  InverseFFTAndWindow(aecm, fft, efw, output);

  return 0;
}

 
