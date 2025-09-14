/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Minimal delay estimator wrapper API (fixed-size, single instance).

#ifndef MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_
#define MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_

#include <stdint.h>

int InitDelayEstimatorFarend(void* handle);
int AddFarSpectrumFix(void* handle, const uint16_t* far_spectrum, int far_q);
int InitDelayEstimator(void* handle);
int DelayEstimatorProcessFix(void* handle, const uint16_t* near_spectrum, int near_q);

#endif  // MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_
