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

typedef int32_t SpectrumType;

typedef struct {
  SpectrumType mean_far_spectrum[PART_LEN1];
  int far_spectrum_initialized;

  int spectrum_size;

  BinaryDelayEstimatorFarend binary_farend;
} DelayEstimatorFarend;

typedef struct {
  SpectrumType mean_near_spectrum[PART_LEN1];
  int near_spectrum_initialized;

  int spectrum_size;

  BinaryDelayEstimator binary_handle;
} DelayEstimator;

// 動的確保APIは削除（固定長・単一インスタンス前提）。

// Initializes the delay estimation far-end state.
void InitBinaryDelayEstimatorFarend();



// Adds the far-end binary spectrum to the internal history buffer.
void AddBinaryFarSpectrum(uint32_t binary_far_spectrum);


// Initializes the near-end binary delay estimator state.
void InitBinaryDelayEstimator();



// Processes a near-end binary spectrum and returns the current delay estimate.
// Return value:
//    - delay                 :  >= 0 - Calculated delay value.
//                              -2    - Insufficient data for estimation.
int ProcessBinarySpectrum(uint32_t binary_near_spectrum);


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
void MeanEstimator(int32_t new_value, int factor, int32_t* mean_value);

// Initializes the singleton far-end delay estimator state.
void InitDelayEstimatorFarend();
// Feeds a new far-end spectrum into the global delay estimator history.
int AddFarSpectrum(const uint16_t* far_spectrum);
// Initializes the singleton near-end delay estimator state.
void InitDelayEstimator();
// Processes the latest near-end spectrum and returns the estimated delay.
int DelayEstimatorProcess(const uint16_t* near_spectrum);

 
