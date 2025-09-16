// Header file including the delay estimator handle used for testing.

#include "delay_estimator.h"
#include "aecm_defines.h"

 

typedef int32_t SpectrumType;

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

 
