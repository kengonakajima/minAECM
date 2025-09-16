#include "delay_estimator_wrapper.h"

#include <string.h>

#include "delay_estimator_internal.h"

 

// 2値スペクトル化で利用する周波数帯域（kBandFirst〜kBandLast）。
// kBandLast - kBandFirst は 32 未満である必要がある。
constexpr int kBandFirst = 12;
constexpr int kBandLast = 43;

__inline uint32_t SetBit(uint32_t in, int pos) {
  uint32_t mask = (1 << pos);
  uint32_t out = (in | mask);

  return out;
}

// 浮動小数点版の指数平均（MeanEstimator と同等の処理）。
// new_value: 新しい値、scale: 平滑化係数 (<1.0)、mean_value: 更新対象。
//
// 閾値スペクトルとの比較で 2 値スペクトルを生成する（固定小数点/float 共通）。
uint32_t BinarySpectrum(const uint16_t* spectrum,
                        SpectrumType* threshold_spectrum,
                        int* threshold_initialized) {
  uint32_t out = 0;

  if (!(*threshold_initialized)) {
    // 初回は入力スペクトルの半分を閾値にセットして収束を早める。
    for (int i = kBandFirst; i <= kBandLast; i++) {
      if (spectrum[i] > 0) {
        int32_t spectrum_q15 = ((int32_t)spectrum[i]) << 15;
        threshold_spectrum[i] = (spectrum_q15 >> 1);
        *threshold_initialized = 1;
      }
    }
  }
  for (int i = kBandFirst; i <= kBandLast; i++) {
    int32_t spectrum_q15 = ((int32_t)spectrum[i]) << 15;
    // 閾値スペクトルを指数平均で更新。
    MeanEstimator(spectrum_q15, 6, &(threshold_spectrum[i]));
    // 閾値と比較して2値化。
    if (spectrum_q15 > threshold_spectrum[i]) {
      out = SetBit(out, i - kBandFirst);
    }
  }

  return out;
}

// Create/Free は廃止。固定長の値型を Init だけで利用する。

int InitDelayEstimatorFarend(void* handle) {
  DelayEstimatorFarend* self = (DelayEstimatorFarend*)handle;

  if (self == NULL) {
    return -1;
  }

  // 遠端側の2値遅延推定部を初期化。
  InitBinaryDelayEstimatorFarend(&self->binary_farend);

  // 平均スペクトルをゼロクリア。
  self->spectrum_size = PART_LEN1;
  memset(self->mean_far_spectrum, 0, sizeof(self->mean_far_spectrum));
  // 初期化フラグをリセット。
  self->far_spectrum_initialized = 0;

  return 0;
}



int AddFarSpectrum(void* handle,
                   const uint16_t* far_spectrum) {
  DelayEstimatorFarend* self = (DelayEstimatorFarend*)handle;
  uint32_t binary_spectrum = 0;

  if (self == NULL) {
    return -1;
  }
  if (far_spectrum == NULL) {
    // 無効な遠端スペクトル。
    return -1;
  }

  // 遠端スペクトルを2値化。
  binary_spectrum = BinarySpectrum(far_spectrum, self->mean_far_spectrum,
                                   &(self->far_spectrum_initialized));
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
  // 2値遅延推定器本体を初期化。
  InitBinaryDelayEstimator(&self->binary_handle);

  // 平均スペクトルをゼロクリア。
  // spectrum_size は Farend に合わせて PART_LEN1（固定）
  self->spectrum_size = PART_LEN1;
  memset(self->mean_near_spectrum, 0, sizeof(self->mean_near_spectrum));
  // 初期化フラグをリセット。
  self->near_spectrum_initialized = 0;

  return 0;
}







int DelayEstimatorProcess(void* handle,
                          const uint16_t* near_spectrum) {
  DelayEstimator* self = (DelayEstimator*)handle;
  uint32_t binary_spectrum = 0;

  if (self == NULL) {
    return -1;
  }
  if (near_spectrum == NULL) {
    // 無効な近端スペクトル。
    return -1;
  }

  // 近端スペクトルを2値化し、遅延推定器へ投入。
  binary_spectrum =
      BinarySpectrum(near_spectrum, self->mean_near_spectrum,
                     &(self->near_spectrum_initialized));

  return ProcessBinarySpectrum(&self->binary_handle, binary_spectrum);
}





 
