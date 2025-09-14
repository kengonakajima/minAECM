/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Header file including the delay estimator handle used for testing.

#ifndef MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_INTERNAL_H_
#define MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_INTERNAL_H_

#include "delay_estimator.h"
#include "aecm_defines.h"

 

typedef union {
  float float_;
  int32_t int32_;
} SpectrumType;

typedef struct {
  // 平均スペクトル（固定長）。
  SpectrumType mean_far_spectrum[PART_LEN1];
  // `mean_far_spectrum` initialization indicator.
  int far_spectrum_initialized;

  int spectrum_size;

  // Far-end part of binary spectrum based delay estimation（値埋め込み）。
  BinaryDelayEstimatorFarend binary_farend;
} DelayEstimatorFarend;

typedef struct {
  // 平均スペクトル（固定長）。
  SpectrumType mean_near_spectrum[PART_LEN1];
  // `mean_near_spectrum` initialization indicator.
  int near_spectrum_initialized;

  int spectrum_size;

  // Binary spectrum based delay estimator（値埋め込み）。
  BinaryDelayEstimator binary_handle;
  // ラッパ側Farendへのポインタ（バインド用）
  DelayEstimatorFarend* farend_wrapper;
} DelayEstimator;

 

#endif  // MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_INTERNAL_H_
