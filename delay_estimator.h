// Performs delay estimation on binary converted spectra.
// The return value is  0 - OK and -1 - Error, unless otherwise stated.

#ifndef MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_H_
#define MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_H_

#include <stdint.h>
#include "aecm_defines.h"

 

static const int32_t kMaxBitCountsQ9 = (32 << 9);  // 32 matching bits in Q9.

typedef struct {
  // 固定長履歴（MAX_DELAY固定）。
  int far_bit_counts[MAX_DELAY];
  uint32_t binary_far_history[MAX_DELAY];
} BinaryDelayEstimatorFarend;

typedef struct {
  // 平滑化済みbitカウント（Q9）と瞬時bitカウント。
  int32_t mean_bit_counts[MAX_DELAY + 1];
  int32_t bit_counts[MAX_DELAY];

  // 近端2値化の履歴（lookahead=0固定なので長さ1）。
  uint32_t binary_near_history[1];

  // 遅延推定の内部状態。
  int32_t minimum_probability;
  int last_delay_probability;
  int last_delay;

  // 堅牢性（ヒストグラム）
  int robust_validation_enabled;
  int allowed_offset;
  int last_candidate_delay;
  int compare_delay;
  int candidate_hits;
  float histogram[MAX_DELAY + 1];
  float last_delay_histogram;

  // Far-end history（外部のFarendに紐付け）。
  BinaryDelayEstimatorFarend* farend;
} BinaryDelayEstimator;

// Releases the memory allocated by
// CreateBinaryDelayEstimatorFarend(...).
// Input:
//    - self              : Pointer to the binary delay estimation far-end
//                          instance which is the return value of
//                          CreateBinaryDelayEstimatorFarend().
//
// 動的確保は行わないため、Freeは不要。

// Allocates the memory needed by the far-end part of the binary delay
// estimation. The memory needs to be initialized separately through
// InitBinaryDelayEstimatorFarend(...).
//
// Inputs:
//      - history_size    : Size of the far-end binary spectrum history.
//
// Return value:
//      - BinaryDelayEstimatorFarend*
//                        : Created `handle`. If the memory can't be allocated
//                          or if any of the input parameters are invalid NULL
//                          is returned.
//
// 動的確保は行わないため、Create/Allocateは不要。

// Re-allocates the buffers.
//
// Inputs:
//      - self            : Pointer to the binary estimation far-end instance
//                          which is the return value of
//                          CreateBinaryDelayEstimatorFarend().
//      - history_size    : Size of the far-end binary spectrum history.
//
// Return value:
//      - history_size    : The history size allocated.
// 固定長バッファのためAllocateは不要。

// Initializes the delay estimation far-end instance created with
// CreateBinaryDelayEstimatorFarend(...).
//
// Input:
//    - self              : Pointer to the delay estimation far-end instance.
//
// Output:
//    - self              : Initialized far-end instance.
//
void InitBinaryDelayEstimatorFarend(BinaryDelayEstimatorFarend* self);



// Adds the binary far-end spectrum to the internal far-end history buffer. This
// spectrum is used as reference when calculating the delay using
// ProcessBinarySpectrum().
//
// Inputs:
//    - self                  : Pointer to the delay estimation far-end
//                              instance.
//    - binary_far_spectrum   : Far-end binary spectrum.
//
// Output:
//    - self                  : Updated far-end instance.
//
void AddBinaryFarSpectrum(BinaryDelayEstimatorFarend* self,
                                 uint32_t binary_far_spectrum);

// Releases the memory allocated by CreateBinaryDelayEstimator(...).
//
// Note that BinaryDelayEstimator utilizes BinaryDelayEstimatorFarend, but does
// not take ownership of it, hence the BinaryDelayEstimator has to be torn down
// before the far-end.
//
// Input:
//    - self              : Pointer to the binary delay estimation instance
//                          which is the return value of
//                          CreateBinaryDelayEstimator().
//
// 動的確保は行わないため、Freeは不要。

// Allocates the memory needed by the binary delay estimation. The memory needs
// to be initialized separately through InitBinaryDelayEstimator(...).
//
// See CreateDelayEstimator(..) in delay_estimator_wrapper.c for detailed
// description.
// 動的確保は行わないため、Createは不要。

// Re-allocates `history_size` dependent buffers. The far-end buffers will be
// updated at the same time if needed.
//
// Input:
//      - self            : Pointer to the binary estimation instance which is
//                          the return value of
//                          CreateBinaryDelayEstimator().
//      - history_size    : Size of the history buffers.
//
// Return value:
//      - history_size    : The history size allocated.
// 固定長バッファのためAllocateは不要。

// Initializes the delay estimation instance created with
// CreateBinaryDelayEstimator(...).
//
// Input:
//    - self              : Pointer to the delay estimation instance.
//
// Output:
//    - self              : Initialized instance.
//
void InitBinaryDelayEstimator(BinaryDelayEstimator* self);



// Estimates and returns the delay between the binary far-end and binary near-
// end spectra. It is assumed the binary far-end spectrum has been added using
// AddBinaryFarSpectrum() prior to this call. The value will be offset by
// the lookahead (i.e. the lookahead should be subtracted from the returned
// value).
//
// Inputs:
//    - self                  : Pointer to the delay estimation instance.
//    - binary_near_spectrum  : Near-end binary spectrum of the current block.
//
// Output:
//    - self                  : Updated instance.
//
// Return value:
//    - delay                 :  >= 0 - Calculated delay value.
//                              -2    - Insufficient data for estimation.
//
int ProcessBinarySpectrum(BinaryDelayEstimator* self,
                                 uint32_t binary_near_spectrum);

// Returns the last calculated delay updated by the function
// ProcessBinarySpectrum(...).
//
// Input:
//    - self                  : Pointer to the delay estimation instance.
//
// Return value:
//    - delay                 :  >= 0 - Last calculated delay value
//                              -2    - Insufficient data for estimation.
//



// Updates the `mean_value` recursively with a step size of 2^-`factor`. This
// function is used internally in the Binary Delay Estimator as well as the
// Fixed point wrapper.
//
// Inputs:
//    - new_value             : The new value the mean should be updated with.
//    - factor                : The step size, in number of right shifts.
//
// Input/Output:
//    - mean_value            : Pointer to the mean value.
//
void MeanEstimatorFix(int32_t new_value,
                             int factor,
                             int32_t* mean_value);

 

#endif  // MODULES_AUDIO_PROCESSING_UTILITY_DELAY_ESTIMATOR_H_
