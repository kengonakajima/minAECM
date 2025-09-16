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
#include <stdio.h>

#include "aecm_core.h"

extern "C" {
#include "ring_buffer.h"
#include "real_fft.h"
}
#include "echo_control_mobile.h"
#include "delay_estimator_wrapper.h"

 
 

// Square root of Hanning window in Q14.
static const ALIGN8_BEG int16_t kSqrtHanning[] ALIGN8_END = {
    0,     399,   798,   1196,  1594,  1990,  2386,  2780,  3172,  3562,  3951,
    4337,  4720,  5101,  5478,  5853,  6224,  6591,  6954,  7313,  7668,  8019,
    8364,  8705,  9040,  9370,  9695,  10013, 10326, 10633, 10933, 11227, 11514,
    11795, 12068, 12335, 12594, 12845, 13089, 13325, 13553, 13773, 13985, 14189,
    14384, 14571, 14749, 14918, 15079, 15231, 15373, 15506, 15631, 15746, 15851,
    15947, 16034, 16111, 16179, 16237, 16286, 16325, 16354, 16373, 16384};

// 近似版の振幅計算は使わず、sqrtベースのみを使用



static void WindowAndFFT(int16_t* fft,
                         const int16_t* time_signal,
                         ComplexInt16* freq_signal) {
  // FFT of signal
  for (int i = 0; i < PART_LEN; i++) {
    // Window time domain signal and insert into real part of
    // transformation array `fft`
    int16_t scaled_time_signal = time_signal[i];
    fft[i] = (int16_t)((scaled_time_signal * kSqrtHanning[i]) >> 14);
    scaled_time_signal = time_signal[i + PART_LEN];
    fft[PART_LEN + i] = (int16_t)(
        (scaled_time_signal * kSqrtHanning[PART_LEN - i]) >> 14);
  }

  // Do forward FFT, then take only the first PART_LEN complex samples,
  // and change signs of the imaginary parts.
  RealForwardFFT(&g_aecm.real_fft, fft, (int16_t*)freq_signal);
  for (int i = 0; i < PART_LEN; i++) {
    freq_signal[i].imag = -freq_signal[i].imag;
  }
}

static void InverseFFTAndWindow(int16_t* fft,
                               ComplexInt16* efw,
                               int16_t* output) {
  // Reuse `efw` for the inverse FFT output after transferring
  // the contents to `fft`.
  int16_t* ifft_out = (int16_t*)efw;

  // Synthesis
  for (int i = 1, j = 2; i < PART_LEN; i += 1, j += 2) {
    fft[j] = efw[i].real;
    fft[j + 1] = -efw[i].imag;
  }
  fft[0] = efw[0].real;
  fft[1] = -efw[0].imag;

  fft[PART_LEN2] = efw[PART_LEN].real;
  fft[PART_LEN2 + 1] = -efw[PART_LEN].imag;

  // Inverse FFT. Keep outCFFT to scale the samples in the next block.
  int outCFFT = RealInverseFFT(&g_aecm.real_fft, fft, ifft_out);
  for (int i = 0; i < PART_LEN; i++) {
    ifft_out[i] = (int16_t)MUL_16_16_RSFT_WITH_ROUND(
        ifft_out[i], kSqrtHanning[i], 14);
    // 固定Q=0のため、出力シフトは outCFFT のみを考慮
    int32_t tmp32no1 = SHIFT_W32((int32_t)ifft_out[i], outCFFT);
    output[i] = (int16_t)SAT(WORD16_MAX,
                                        tmp32no1 + g_aecm.outBuf[i],
                                        WORD16_MIN);

    tmp32no1 = (ifft_out[PART_LEN + i] *
                kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = SHIFT_W32(tmp32no1, outCFFT);
    g_aecm.outBuf[i] = (int16_t)SAT(WORD16_MAX, tmp32no1,
                                              WORD16_MIN);
  }

  // Copy the current block to the old position
  // (aecm->outBuf is shifted elsewhere)
  memcpy(g_aecm.xBuf, g_aecm.xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.dBufNoisy, g_aecm.dBufNoisy + PART_LEN,
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
//
static void TimeToFrequencyDomain(const int16_t* time_signal,
                                  ComplexInt16* freq_signal,
                                  uint16_t* freq_signal_abs,
                                  uint32_t* freq_signal_sum_abs) {
  int16_t fft[PART_LEN4];

  WindowAndFFT(fft, time_signal, freq_signal);

  // Extract imaginary and real part, calculate the magnitude for
  // all frequency bins
  freq_signal[0].imag = 0;
  freq_signal[PART_LEN].imag = 0;
  freq_signal_abs[0] = (uint16_t)ABS_W16(freq_signal[0].real);
  freq_signal_abs[PART_LEN] =
      (uint16_t)ABS_W16(freq_signal[PART_LEN].real);
  (*freq_signal_sum_abs) =
      (uint32_t)(freq_signal_abs[0]) + (uint32_t)(freq_signal_abs[PART_LEN]);

  for (int i = 1; i < PART_LEN; i++) {
    if (freq_signal[i].real == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].imag);
    } else if (freq_signal[i].imag == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].real);
    } else {
      int16_t abs_real = ABS_W16(freq_signal[i].real);
      int16_t abs_imag = ABS_W16(freq_signal[i].imag);
      int32_t sq_real = abs_real * abs_real;
      int32_t sq_imag = abs_imag * abs_imag;
      int32_t sum_sq = AddSatW32(sq_real, sq_imag);
      int32_t mag = SqrtFloor(sum_sq);
      freq_signal_abs[i] = (uint16_t)mag;
    }
    (*freq_signal_sum_abs) += (uint32_t)freq_signal_abs[i];
  }

}

 

int ProcessBlock(const int16_t* farend,
                            const int16_t* nearend,
                            int16_t* output) {
  // 周波数領域バッファ
  ComplexInt16 dfw[PART_LEN2];
  // 近端/遠端スペクトルの絶対値
  uint16_t xfa[PART_LEN1];
  uint16_t dfaNoisy[PART_LEN1];
  // エコー推定（Qスケール固定）
  int32_t echoEst32[PART_LEN1];

  // Determine startup state. There are three states:
  // (0) the first CONV_LEN blocks
  // (1) another CONV_LEN blocks
  // (2) the rest

  if (g_aecm.startupState < 2) {
    g_aecm.startupState =
        (g_aecm.totCount >= CONV_LEN) + (g_aecm.totCount >= CONV_LEN2);
  }
  // END: Determine startup state

  // Buffer near and far end signals (time-domain frame buffering)
  memcpy(g_aecm.xBuf + PART_LEN, farend, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.dBufNoisy + PART_LEN, nearend, sizeof(int16_t) * PART_LEN);

  // Transform far end signal X(k) = FFT{x(n)} (|X|とΣ|X|)
  uint32_t xfaSum = 0;
  TimeToFrequencyDomain(g_aecm.xBuf, dfw, xfa, &xfaSum);

  // Transform noisy near end signal D(k) = FFT{d(n)} (|D|とΣ|D|)
  uint32_t dfaNoisySum = 0;
  TimeToFrequencyDomain(g_aecm.dBufNoisy, dfw, dfaNoisy, &dfaNoisySum);
  g_aecm.dfaNoisyQDomainOld = g_aecm.dfaNoisyQDomain;
  g_aecm.dfaNoisyQDomain = 0;

  
  g_aecm.dfaCleanQDomainOld = g_aecm.dfaNoisyQDomainOld;
  g_aecm.dfaCleanQDomain = g_aecm.dfaNoisyQDomain;

  // Update far-history and estimate delay via binary spectra matching
  UpdateFarHistory(xfa);
  if (AddFarSpectrum(&g_aecm.delay_estimator_farend, xfa) == -1) {
    return -1;
  }
  int delay = DelayEstimatorProcess(&g_aecm.delay_estimator, dfaNoisy);
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    // If the delay is unknown, we assume zero.
    // NOTE: this will have to be adjusted if we ever add lookahead.
    delay = 0;
  }

  // Align far spectrum according to estimated delay
  const uint16_t* far_spectrum_ptr = AlignedFarend(delay);
  if (far_spectrum_ptr == NULL) {
    return -1;
  }

  // Update energy logs (log |X|, log |Y_hat|) for VAD/thresholds
  CalcEnergies(far_spectrum_ptr, dfaNoisySum,
                          echoEst32);

  // NLMS step size μ based on far energy dynamics
  int16_t mu = CalcStepSize();

  // Increment processed-block counter
  g_aecm.totCount++;

  // This is the channel estimation algorithm.
  // It is base on NLMS but has a variable step length,
  // which was calculated above.
  UpdateChannel(far_spectrum_ptr, 0 /*far_q*/, dfaNoisy, mu,
                           echoEst32);
  int16_t supGain = CalcSuppressionGain();

  // Calculate Wiener filter H(k)
  uint16_t* ptrDfaClean = dfaNoisy;  // |D_clean| proxy
  int16_t hnl[PART_LEN1];
  int16_t numPosCoef = 0;
  for (int i = 0; i < PART_LEN1; i++) {
    // Far end signal through channel estimate in Q8
    // How much can we shift right to preserve resolution
    int32_t tmp32no1 = echoEst32[i] - g_aecm.echoFilt[i];
    g_aecm.echoFilt[i] += (int32_t)(((int64_t)tmp32no1 * 50) >> 8);  // ζ = 50/256 update (EMA)

    int16_t zeros32 = NormW32(g_aecm.echoFilt[i]) + 1;
    int16_t zeros16 = NormW16(supGain) + 1;
    uint32_t echoEst32Gained;
    int16_t resolutionDiff;
    if (zeros32 + zeros16 > 16) {
      // Multiplication is safe
      // Result in Q(RESOLUTION_CHANNEL + RESOLUTION_SUPGAIN)
      echoEst32Gained = UMUL_32_16((uint32_t)g_aecm.echoFilt[i], (uint16_t)supGain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
    } else {
      int16_t tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff =
          14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      if (zeros32 > tmp16no1) {
        echoEst32Gained = UMUL_32_16((uint32_t)g_aecm.echoFilt[i],
                                                supGain >> tmp16no1);
      } else {
        // Result in Q(RESOLUTION_CHANNEL + RESOLUTION_SUPGAIN - tmp16no1)
        echoEst32Gained = (g_aecm.echoFilt[i] >> tmp16no1) * supGain;
      }
    }

    zeros16 = NormW16(g_aecm.nearFilt[i]);
    int16_t dfa_clean_q_domain_diff = g_aecm.dfaCleanQDomain - g_aecm.dfaCleanQDomainOld;
    int16_t qDomainDiff;
    int16_t tmp16no1;
    int16_t tmp16no2;
    if (zeros16 < dfa_clean_q_domain_diff && g_aecm.nearFilt[i]) {
      tmp16no1 = g_aecm.nearFilt[i] * (1 << zeros16);
      qDomainDiff = zeros16 - dfa_clean_q_domain_diff;
      tmp16no2 = ptrDfaClean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = dfa_clean_q_domain_diff < 0
                     ? g_aecm.nearFilt[i] >> -dfa_clean_q_domain_diff
                     : g_aecm.nearFilt[i] * (1 << dfa_clean_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = ptrDfaClean[i];
    }
    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
    tmp16no2 += tmp16no1;
    zeros16 = NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      g_aecm.nearFilt[i] = WORD16_MAX;
    } else {
      g_aecm.nearFilt[i] = qDomainDiff < 0 ? tmp16no2 * (1 << -qDomainDiff)
                                           : tmp16no2 >> qDomainDiff;
    }

    // Wiener filter coefficients, resulting hnl in Q14 (H(k) = 1 - |Ŷ|/|D|)
    if (echoEst32Gained == 0) {
      hnl[i] = ONE_Q14;
    } else if (g_aecm.nearFilt[i] == 0) {
      hnl[i] = 0;
    } else {
      // Multiply the suppression gain and apply rounding
      echoEst32Gained += (uint32_t)(g_aecm.nearFilt[i] >> 1);
      uint32_t tmpU32 = DivU32U16(echoEst32Gained, (uint16_t)g_aecm.nearFilt[i]);

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
  // 上帯域のゲインが下帯域を上回らないよう制限。
  // TODO(bjornv): Investigate if the scaling of hnl[i] below can cause
  //               speech distortion in double-talk.
  for (int i = 0; i < PART_LEN1; i++) {
    hnl[i] = (int16_t)((hnl[i] * hnl[i]) >> 14);
  }

  const int kMinPrefBand = 4;   // Prefilter band (4..24) per spec
  const int kMaxPrefBand = 24;
  int32_t avgHnl32 = 0;
  for (int i = kMinPrefBand; i <= kMaxPrefBand; i++) {
    avgHnl32 += (int32_t)hnl[i];
  }
  avgHnl32 /= (kMaxPrefBand - kMinPrefBand + 1);

  for (int i = kMaxPrefBand; i < PART_LEN1; i++) {
    if (hnl[i] > (int16_t)avgHnl32) {
      hnl[i] = (int16_t)avgHnl32;
    }
  }

  // NLPゲイン計算と乗算を常時実行。
  ComplexInt16 efw[PART_LEN2];  // Error spectrum E(k)
  for (int i = 0; i < PART_LEN1; i++) {
    // Truncate values close to zero and one.
    if (hnl[i] > NLP_COMP_HIGH) {
      hnl[i] = ONE_Q14;
    } else if (hnl[i] < NLP_COMP_LOW) {
      hnl[i] = 0;
    }

    // Remove outliers
    int16_t nlpGain = (numPosCoef < 3) ? 0 : ONE_Q14;  // double-talk guard

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
  // 逆FFT用の作業バッファは使用直前に確保
  int16_t fft[PART_LEN4 + 2];  // +2 to make a loop safe.
  InverseFFTAndWindow(fft, efw, output);

  // Debug: print startupState periodically for education/metrics
  {
    static int dbg_ss_counter = 0;
    dbg_ss_counter++;
    if (dbg_ss_counter % 100 == 0) {
      fprintf(stderr, "[AECM] block=%d startupState=%d\n",
              dbg_ss_counter, (int)g_aecm.startupState);
    }
  }

  return 0;
}

 
