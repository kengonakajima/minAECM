// Performs delay estimation on binary converted spectra.
// The return value is  0 - OK and -1 - Error, unless otherwise stated.

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

  // 堅牢性（ヒストグラム）は常時有効（フラグ廃止）
  int allowed_offset;
  int last_candidate_delay;
  int compare_delay;
  int candidate_hits;
  float histogram[MAX_DELAY + 1];
  float last_delay_histogram;

  // Far-end history（外部のFarendに紐付け）。
  BinaryDelayEstimatorFarend* farend;
} BinaryDelayEstimator;

// 動的確保APIは削除（固定長・単一インスタンス前提）。

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
void MeanEstimator(int32_t new_value,
                   int factor,
                   int32_t* mean_value);

 
