/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Performs delay estimation on block by block basis.
// The return value is  0 - OK and -1 - Error, unless otherwise stated.

#ifndef MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_
#define MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_

#include <stdint.h>

 
// Farend/Estimatorは単一インスタンス前提で外部確保不要。Initのみ提供。

// Initializes the far-end part of the delay estimation instance returned by
// CreateDelayEstimatorFarend(...)
int InitDelayEstimatorFarend(void* handle);

// Soft resets the far-end part of the delay estimation instance returned by
// CreateDelayEstimatorFarend(...).
// Input:
//      - delay_shift   : The amount of blocks to shift history buffers.

// Adds the far-end spectrum to the far-end history buffer. This spectrum is
// used as reference when calculating the delay using
// ProcessSpectrum().
//
// Inputs:
//    - far_spectrum    : Far-end spectrum.
//    - spectrum_size   : The size of the data arrays (same for both far- and
//                        near-end).
//    - far_q           : The Q-domain of the far-end data.
//
// Output:
//    - handle          : Updated far-end instance.
//
int AddFarSpectrumFix(void* handle,
                             const uint16_t* far_spectrum,
                             int far_q);

// See AddFarSpectrumFix() for description.

// Initializes the delay estimation instance returned by
// CreateDelayEstimator(...)
int InitDelayEstimator(void* handle);

// Soft resets the delay estimation instance returned by
// CreateDelayEstimator(...)
// Input:
//      - delay_shift   : The amount of blocks to shift history buffers.
//
// Return value:
//      - actual_shifts : The actual number of shifts performed.

// Sets the effective `history_size` used. Valid values from 2. We simply need
// at least two delays to compare to perform an estimate. If `history_size` is
// changed, buffers are reallocated filling in with zeros if necessary.
// Note that changing the `history_size` affects both buffers in far-end and
// near-end. Hence it is important to change all DelayEstimators that use the
// same reference far-end, to the same `history_size` value.
// Inputs:
//  - handle            : Pointer to the delay estimation instance.
//  - history_size      : Effective history size to be used.
// Return value:
//  - new_history_size  : The new history size used. If the memory was not able
//                        to be allocated 0 is returned.


// Enables/Disables a robust validation functionality in the delay estimation.
// This is by default set to disabled at create time.  The state is preserved
// over a reset.
// Inputs:
//      - handle        : Pointer to the delay estimation instance.
//      - enable        : Enable (1) or disable (0) this feature.


// Estimates and returns the delay between the far-end and near-end blocks. The
// value will be offset by the lookahead (i.e. the lookahead should be
// subtracted from the returned value).
// Inputs:
//      - handle        : Pointer to the delay estimation instance.
//      - near_spectrum : Pointer to the near-end spectrum data of the current
//                        block.
//      - spectrum_size : The size of the data arrays (same for both far- and
//                        near-end).
//      - near_q        : The Q-domain of the near-end data.
//
// Output:
//      - handle        : Updated instance.
//
// Return value:
//      - delay         :  >= 0 - Calculated delay value.
//                        -1    - Error.
//                        -2    - Insufficient data for estimation.
int DelayEstimatorProcessFix(void* handle,
                                    const uint16_t* near_spectrum,
                                    int near_q);

// See DelayEstimatorProcessFix() for description.

// Returns the last calculated delay updated by the function
// DelayEstimatorProcess(...).
//
// Input:
//      - handle        : Pointer to the delay estimation instance.
//
// Return value:
//      - delay         : >= 0  - Last calculated delay value.
//                        -1    - Error.
//                        -2    - Insufficient data for estimation.


// Returns the estimation quality/probability of the last calculated delay
// updated by the function DelayEstimatorProcess(...). The estimation
// quality is a value in the interval [0, 1]. The higher the value, the better
// the quality.
//
// Return value:
//      - delay_quality : >= 0  - Estimation quality of last calculated delay.


 

#endif  // MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_WRAPPER_H_
