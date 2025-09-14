/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "delay_estimator_wrapper.h"

#include <string.h>

#include "delay_estimator.h"
#include "delay_estimator_internal.h"
#include "aecm_defines.h"

 

// Only bit `kBandFirst` through bit `kBandLast` are processed and
// `kBandFirst` - `kBandLast` must be < 32.
constexpr int kBandFirst = 12;
constexpr int kBandLast = 43;

static __inline uint32_t SetBit(uint32_t in, int pos) {
  uint32_t mask = (1 << pos);
  uint32_t out = (in | mask);

  return out;
}

// Calculates the mean recursively. Same version as MeanEstimatorFix(),
// but for float.
//
// Inputs:
//    - new_value             : New additional value.
//    - scale                 : Scale for smoothing (should be less than 1.0).
//
// Input/Output:
//    - mean_value            : Pointer to the mean value for updating.
//
// Computes the binary spectrum by comparing the input `spectrum` with a
// `threshold_spectrum`. Float and fixed point versions.
//
// Inputs:
//      - spectrum            : Spectrum of which the binary spectrum should be
//                              calculated.
//      - threshold_spectrum  : Threshold spectrum with which the input
//                              spectrum is compared.
// Return:
//      - out                 : Binary spectrum.
//
static uint32_t BinarySpectrumFix(const uint16_t* spectrum,
                                  SpectrumType* threshold_spectrum,
                                  int q_domain,
                                  int* threshold_initialized) {
  int i = kBandFirst;
  uint32_t out = 0;

  // expect q_domain < 16

  if (!(*threshold_initialized)) {
    // Set the `threshold_spectrum` to half the input `spectrum` as starting
    // value. This speeds up the convergence.
    for (i = kBandFirst; i <= kBandLast; i++) {
      if (spectrum[i] > 0) {
        // Convert input spectrum from Q(`q_domain`) to Q15.
        int32_t spectrum_q15 = ((int32_t)spectrum[i]) << (15 - q_domain);
        threshold_spectrum[i].int32_ = (spectrum_q15 >> 1);
        *threshold_initialized = 1;
      }
    }
  }
  for (i = kBandFirst; i <= kBandLast; i++) {
    // Convert input spectrum from Q(`q_domain`) to Q15.
    int32_t spectrum_q15 = ((int32_t)spectrum[i]) << (15 - q_domain);
    // Update the `threshold_spectrum`.
    MeanEstimatorFix(spectrum_q15, 6, &(threshold_spectrum[i].int32_));
    // Convert `spectrum` at current frequency bin to a binary value.
    if (spectrum_q15 > threshold_spectrum[i].int32_) {
      out = SetBit(out, i - kBandFirst);
    }
  }

  return out;
}

// Create/Freeは廃止。固定長の値型をInitのみで使用する。

int InitDelayEstimatorFarend(void* handle) {
  DelayEstimatorFarend* self = (DelayEstimatorFarend*)handle;

  if (self == NULL) {
    return -1;
  }

  // Initialize far-end part of binary delay estimator.
  InitBinaryDelayEstimatorFarend(&self->binary_farend);

  // Set averaged far and near end spectra to zero.
  self->spectrum_size = PART_LEN1;
  memset(self->mean_far_spectrum, 0, sizeof(self->mean_far_spectrum));
  // Reset initialization indicators.
  self->far_spectrum_initialized = 0;

  return 0;
}



int AddFarSpectrumFix(void* handle,
                             const uint16_t* far_spectrum,
                             int far_q) {
  DelayEstimatorFarend* self = (DelayEstimatorFarend*)handle;
  uint32_t binary_spectrum = 0;

  if (self == NULL) {
    return -1;
  }
  if (far_spectrum == NULL) {
    // Empty far end spectrum.
    return -1;
  }
  if (far_q > 15) {
    // If `far_q` is larger than 15 we cannot guarantee no wrap around.
    return -1;
  }

  // Get binary spectrum.
  binary_spectrum = BinarySpectrumFix(far_spectrum, self->mean_far_spectrum,
                                      far_q, &(self->far_spectrum_initialized));
  AddBinaryFarSpectrum(&self->binary_farend, binary_spectrum);

  return 0;
}

int InitDelayEstimator(void* handle) {
  DelayEstimator* self = (DelayEstimator*)handle;

  if (self == NULL) {
    return -1;
  }

  // Farend（ラッパ）をバイナリエンジンへ接続
  if (self->farend_wrapper) {
    self->binary_handle.farend = &self->farend_wrapper->binary_farend;
  }
  // Initialize binary delay estimator.
  InitBinaryDelayEstimator(&self->binary_handle);

  // Set averaged far and near end spectra to zero.
  // spectrum_size は Farend に合わせて PART_LEN1（固定）
  self->spectrum_size = PART_LEN1;
  memset(self->mean_near_spectrum, 0, sizeof(self->mean_near_spectrum));
  // Reset initialization indicators.
  self->near_spectrum_initialized = 0;

  return 0;
}







int DelayEstimatorProcessFix(void* handle,
                                    const uint16_t* near_spectrum,
                                    int near_q) {
  DelayEstimator* self = (DelayEstimator*)handle;
  uint32_t binary_spectrum = 0;

  if (self == NULL) {
    return -1;
  }
  if (near_spectrum == NULL) {
    // Empty near end spectrum.
    return -1;
  }
  if (near_q > 15) {
    // If `near_q` is larger than 15 we cannot guarantee no wrap around.
    return -1;
  }

  // Get binary spectra.
  binary_spectrum =
      BinarySpectrumFix(near_spectrum, self->mean_near_spectrum, near_q,
                        &(self->near_spectrum_initialized));

  return ProcessBinarySpectrum(&self->binary_handle, binary_spectrum);
}





 
