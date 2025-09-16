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
                                        tmp32no1 + g_aecm.eOverlapBuf[i],
                                        WORD16_MIN);

    tmp32no1 = (ifft_out[PART_LEN + i] *
                kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = SHIFT_W32(tmp32no1, outCFFT);
    g_aecm.eOverlapBuf[i] = (int16_t)SAT(WORD16_MAX, tmp32no1,
                                              WORD16_MIN);
  }

  // Copy the current block to the old position
  // (aecm->eOverlapBuf is shifted elsewhere)
  memcpy(g_aecm.xBuf, g_aecm.xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.yBuf, g_aecm.yBuf + PART_LEN,
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

 

int ProcessBlock(const int16_t* x_block,
                            const int16_t* y_block,
                            int16_t* e_block) {
  // 周波数領域バッファ（Y(k) の複素成分）
  ComplexInt16 Y_freq[PART_LEN2];
  // |X(k)|, |Y(k)| の絶対値スペクトル
  uint16_t X_mag[PART_LEN1];
  uint16_t Y_mag[PART_LEN1];
  // |Ŝ(k)|: 予測エコー振幅（チャネル通過後）
  int32_t S_mag[PART_LEN1];

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
  memcpy(g_aecm.xBuf + PART_LEN, x_block, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.yBuf + PART_LEN, y_block, sizeof(int16_t) * PART_LEN);

  // Transform far end signal X(k) = FFT{x(n)} (|X|とΣ|X|)
  uint32_t X_mag_sum = 0;
  TimeToFrequencyDomain(g_aecm.xBuf, Y_freq, X_mag, &X_mag_sum);

  // Transform noisy near end signal D(k) = FFT{d(n)} (|D|とΣ|D|)
  uint32_t Y_mag_sum = 0;
  TimeToFrequencyDomain(g_aecm.yBuf, Y_freq, Y_mag, &Y_mag_sum);
  g_aecm.dfaNoisyQDomainOld = g_aecm.dfaNoisyQDomain;
  g_aecm.dfaNoisyQDomain = 0;

  
  g_aecm.dfaCleanQDomainOld = g_aecm.dfaNoisyQDomainOld;
  g_aecm.dfaCleanQDomain = g_aecm.dfaNoisyQDomain;

  // Update far-history and estimate delay via binary spectra matching
  UpdateFarHistory(X_mag);
  if (AddFarSpectrum(&g_aecm.delay_estimator_farend, X_mag) == -1) {
    return -1;
  }
  int delay = DelayEstimatorProcess(&g_aecm.delay_estimator, Y_mag);
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    // If the delay is unknown, we assume zero.
    // NOTE: this will have to be adjusted if we ever add lookahead.
    delay = 0;
  }

  // Align far spectrum according to estimated delay
  const uint16_t* X_mag_aligned = AlignedFarX(delay);
  if (X_mag_aligned == NULL) {
    return -1;
  }

  // Update energy logs (log |X|, log |Y_hat|) for VAD/thresholds
  CalcEnergies(X_mag_aligned, Y_mag_sum,
                          S_mag);

  // NLMS step size μ based on far energy dynamics
  int16_t mu = CalcStepSize();

  // Increment processed-block counter
  g_aecm.totCount++;

  // This is the channel estimation algorithm.
  // It is base on NLMS but has a variable step length,
  // which was calculated above.
  UpdateChannel(X_mag_aligned, 0 /*x_q*/, Y_mag, mu,
                           S_mag);
  int16_t gGain = CalcSuppressionGain();

  // Calculate Wiener filter H(k)
  uint16_t* Y_mag_clean = Y_mag;  // |Y_clean(k)| proxy
  int16_t H_gain[PART_LEN1];
  int16_t numPosCoef = 0;
  for (int i = 0; i < PART_LEN1; i++) {
    // Far end signal through channel estimate in Q8
    // How much can we shift right to preserve resolution
    // Ŷ(k) の緩和更新（g_aecm.sMagSmooth ≒ 過去の |Ŷ(k)|）
    int32_t tmp32no1 = S_mag[i] - g_aecm.sMagSmooth[i];
    g_aecm.sMagSmooth[i] += (int32_t)(((int64_t)tmp32no1 * 50) >> 8);  // ζ = 50/256 update (EMA)

    int16_t zeros32 = NormW32(g_aecm.sMagSmooth[i]) + 1;
    int16_t zeros16 = NormW16(gGain) + 1;
    uint32_t sMagGained;
    int16_t resolutionDiff;
    if (zeros32 + zeros16 > 16) {
      // Multiplication is safe
      // Result in Q(RESOLUTION_CHANNEL + RESOLUTION_SUPGAIN) for |Ŷ(k)|·G(k)
      sMagGained = UMUL_32_16((uint32_t)g_aecm.sMagSmooth[i], (uint16_t)gGain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
    } else {
      int16_t tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff =
          14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      if (zeros32 > tmp16no1) {
        sMagGained = UMUL_32_16((uint32_t)g_aecm.sMagSmooth[i],
                                                gGain >> tmp16no1);
      } else {
        // Result in Q(RESOLUTION_CHANNEL + RESOLUTION_SUPGAIN - tmp16no1)
        sMagGained = (g_aecm.sMagSmooth[i] >> tmp16no1) * gGain;
      }
    }

    zeros16 = NormW16(g_aecm.yMagSmooth[i]);
    int16_t y_mag_q_domain_diff =
        g_aecm.dfaCleanQDomain - g_aecm.dfaCleanQDomainOld;
    int16_t qDomainDiff;
    int16_t tmp16no1;
    int16_t tmp16no2;
    if (zeros16 < y_mag_q_domain_diff && g_aecm.yMagSmooth[i]) {
      tmp16no1 = g_aecm.yMagSmooth[i] * (1 << zeros16);
      qDomainDiff = zeros16 - y_mag_q_domain_diff;
      tmp16no2 = Y_mag_clean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = y_mag_q_domain_diff < 0
                     ? g_aecm.yMagSmooth[i] >> -y_mag_q_domain_diff
                     : g_aecm.yMagSmooth[i] * (1 << y_mag_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = Y_mag_clean[i];
    }
    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
    tmp16no2 += tmp16no1;
    zeros16 = NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      g_aecm.yMagSmooth[i] = WORD16_MAX;
    } else {
      g_aecm.yMagSmooth[i] = qDomainDiff < 0 ? tmp16no2 * (1 << -qDomainDiff)
                                           : tmp16no2 >> qDomainDiff;
    }

    // Wienerフィルタ係数 H(k) = 1 - |Ŷ(k)| / |D(k)| （結果は Q14）
    if (sMagGained == 0) {
      H_gain[i] = ONE_Q14;
    } else if (g_aecm.yMagSmooth[i] == 0) {
      H_gain[i] = 0;
    } else {
      // Multiply the suppression gain and apply rounding
      sMagGained += (uint32_t)(g_aecm.yMagSmooth[i] >> 1);
      uint32_t tmpU32 = DivU32U16(sMagGained, (uint16_t)g_aecm.yMagSmooth[i]);

      // Current resolution is
      // Q-(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN- max(0,17-zeros16- zeros32))
      // Make sure we are in Q14
      tmp32no1 = (int32_t)SHIFT_W32(tmpU32, resolutionDiff);
      if (tmp32no1 > ONE_Q14) {
        H_gain[i] = 0;
      } else if (tmp32no1 < 0) {
        H_gain[i] = ONE_Q14;
      } else {
        // 1 - S_mag / Y_mag
        H_gain[i] = ONE_Q14 - (int16_t)tmp32no1;
        if (H_gain[i] < 0) {
          H_gain[i] = 0;
        }
      }
    }
    if (H_gain[i]) {
      numPosCoef++;
    }
  }
  // 上帯域のゲインが下帯域を上回らないよう制限。
  // TODO(bjornv): Investigate if the scaling of H_gain[i] below can cause
  //               speech distortion in double-talk.
  for (int i = 0; i < PART_LEN1; i++) {
    H_gain[i] = (int16_t)((H_gain[i] * H_gain[i]) >> 14);
  }

  const int kMinPrefBand = 4;   // Prefilter band (4..24) per spec
  const int kMaxPrefBand = 24;
  int32_t avgH32 = 0;
  for (int i = kMinPrefBand; i <= kMaxPrefBand; i++) {
    avgH32 += (int32_t)H_gain[i];
  }
  avgH32 /= (kMaxPrefBand - kMinPrefBand + 1);

  for (int i = kMaxPrefBand; i < PART_LEN1; i++) {
    if (H_gain[i] > (int16_t)avgH32) {
      H_gain[i] = (int16_t)avgH32;
    }
  }

  // NLPゲイン計算と乗算を常時実行。
  ComplexInt16 E_freq[PART_LEN2];  // 誤差複素スペクトル E(k) = H(k)·Y(k)
  for (int i = 0; i < PART_LEN1; i++) {
    // Truncate values close to zero and one.
    if (H_gain[i] > NLP_COMP_HIGH) {
      H_gain[i] = ONE_Q14;
    } else if (H_gain[i] < NLP_COMP_LOW) {
      H_gain[i] = 0;
    }

    // Remove outliers
    int16_t nlpGain = (numPosCoef < 3) ? 0 : ONE_Q14;  // double-talk guard

    // NLP
    if ((H_gain[i] == ONE_Q14) && (nlpGain == ONE_Q14)) {
      H_gain[i] = ONE_Q14;
    } else {
      H_gain[i] = (int16_t)((H_gain[i] * nlpGain) >> 14);
    }

    // multiply with Wiener coefficients
    E_freq[i].real = (int16_t)(
        MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].real, H_gain[i], 14));
    E_freq[i].imag = (int16_t)(
        MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].imag, H_gain[i], 14));
  }
  // 逆FFT用の作業バッファは使用直前に確保
  int16_t fft[PART_LEN4 + 2];  // +2 to make a loop safe.
  InverseFFTAndWindow(fft, E_freq, e_block);

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

 
