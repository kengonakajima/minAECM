#include "delay_estimator.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <algorithm>

 
 

// スケーリングの右シフト回数は遠端2値スペクトルのビット数に線形依存する。
// 
static const int kShiftsAtZero = 13;  // 2値スペクトルが 0 のときの右シフト量。
static const int kShiftsLinearSlope = 3;

static const int32_t kProbabilityOffset = 1024;      // Q9 で 2.
static const int32_t kProbabilityLowerLimit = 8704;  // Q9 で 17.
static const int32_t kProbabilityMinSpread = 2816;   // Q9 で 5.5.

// ロバスト検証関連の定数
static const float kHistogramMax = 3000.f;
static const float kLastHistogramMax = 250.f;
static const float kMinHistogramThreshold = 1.5f;
static const int kMinRequiredHits = 10;
static const int kMaxHitsWhenPossiblyNonCausal = 10;
static const int kMaxHitsWhenPossiblyCausal = 1000;
static const float kQ14Scaling = 1.f / (1 << 14);  // 2^14 で割って Q0 に変換。
static const float kFractionSlope = 0.05f;
static const float kMinFractionWhenPossiblyCausal = 0.5f;
static const float kMinFractionWhenPossiblyNonCausal = 0.25f;

 

// 32bit ワードに含まれるビット数を数えて返す。
static int BitCount(uint32_t u32) {
  uint32_t tmp =
      u32 - ((u32 >> 1) & 033333333333) - ((u32 >> 2) & 011111111111);
  tmp = ((tmp + (tmp >> 3)) & 030707070707);
  tmp = (tmp + (tmp >> 6));
  tmp = (tmp + (tmp >> 12) + (tmp >> 24)) & 077;

  return ((int)tmp);
}

// `binary_vector` を `binary_matrix` の各行と比較し、
// 行ごとに一致するビット数を数える。
//
// 入力:
//      - binary_vector     : binary "vector" stored in a long
//      - binary_matrix     : binary "matrix" stored as a vector of long
//      - matrix_size       : size of binary "matrix"
//
// 出力:
//      - bit_counts        : "Vector" stored as a long, containing for each
//                            row the number of times the matrix row and the
//                            input vector have the same value
//
static void BitCountComparison(uint32_t binary_vector,
                               const uint32_t* binary_matrix,
                               int matrix_size,
                               int32_t* bit_counts) {
  // `binary_vector` を各行と比較
  for (int n = 0; n < matrix_size; n++) {
    bit_counts[n] = (int32_t)BitCount(binary_vector ^ binary_matrix[n]);
  }
}

// HistogramBasedValidation() に必要な統計量を更新する。
// この関数は HistogramBasedValidation() より先に呼び出す必要がある。
// 更新される統計量は以下の通り。
//  1. the number of `candidate_hits`, which states for how long we have had the
//     same `candidate_delay`
//  2. the `histogram` of candidate delays over time.  This histogram is
//     weighted with respect to a reliability measure and time-varying to cope
//     with possible delay shifts.
// 詳細はコメント内コード参照。
//
// 入力:
//  - candidate_delay   : 検証対象の遅延。
//  - valley_depth_q14  : コスト関数の谷の深さ (Q14)。候補と最悪値の差。
//  - valley_level_q14  : コスト関数の最小値 (Q14)。
static void UpdateRobustValidationStatistics(BinaryDelayEstimator* self,
                                             int candidate_delay,
                                             int32_t valley_depth_q14,
                                             int32_t valley_level_q14) {
  const float valley_depth = valley_depth_q14 * kQ14Scaling;
  float decrease_in_last_set = valley_depth;
  const int max_hits_for_slow_change = (candidate_delay < self->last_delay)
                                           ? kMaxHitsWhenPossiblyNonCausal
                                           : kMaxHitsWhenPossiblyCausal;

  // 履歴サイズが同じであることを期待。
  // 新しい候補が出た場合は `candidate_hits` をリセット。
  if (candidate_delay != self->last_candidate_delay) {
    self->candidate_hits = 0;
    self->last_candidate_delay = candidate_delay;
  }
  self->candidate_hits++;

  // `histogram` はビンごとに異なる更新を行う。
  // 1. `candidate_delay` のヒストグラムビンは信頼度 (valley_depth) に応じて増加させる。
  //    `valley_depth`, which is a simple measure of how reliable the
  //    `candidate_delay` is.  The histogram is not increased above
  //    `kHistogramMax`.
  self->histogram[candidate_delay] += valley_depth;
  if (self->histogram[candidate_delay] > kHistogramMax) {
    self->histogram[candidate_delay] = kHistogramMax;
  }
  // 2. The histogram bins in the neighborhood of `candidate_delay` are
  //    unaffected.  The neighborhood is defined as x + {-2, -1, 0, 1}.
  // 3. The histogram bins in the neighborhood of `last_delay` are decreased
  //    with `decrease_in_last_set`.  This value equals the difference between
  //    the cost function values at the locations `candidate_delay` and
  //    `last_delay` until we reach `max_hits_for_slow_change` consecutive hits
  //    at the `candidate_delay`.  If we exceed this amount of hits the
  //    `candidate_delay` is a "potential" candidate and we start decreasing
  //    these histogram bins more rapidly with `valley_depth`.
  if (self->candidate_hits < max_hits_for_slow_change) {
    decrease_in_last_set =
        (self->mean_bit_counts[self->compare_delay] - valley_level_q14) *
        kQ14Scaling;
  }
  // 4. その他のビンは valley_depth で減少させる。
  for (int i = 0; i < MAX_DELAY; ++i) {
    int is_in_last_set = (i >= self->last_delay - 2) &&
                         (i <= self->last_delay + 1) && (i != candidate_delay);
    int is_in_candidate_set =
        (i >= candidate_delay - 2) && (i <= candidate_delay + 1);
    self->histogram[i] -=
        decrease_in_last_set * is_in_last_set +
        valley_depth * (!is_in_last_set && !is_in_candidate_set);
    // 5. ビンは 0 未満にならないよう制限。
    if (self->histogram[i] < 0) {
      self->histogram[i] = 0;
    }
  }
}

// ProcessBinarySpectrum() で求めた `candidate_delay` を検証する。
// 一致回数のカウントと修正ヒストグラムの組み合わせで評価する。
// 簡単に言えば、ヒストグラム上もっとも尤度が高ければ候補を採用（1）。
// 
// 例外条件も存在する。
//  1. If the `candidate_delay` < `last_delay` it can be that we are in a
//     non-causal state, breaking a possible echo control algorithm.  Hence, we
//     open up for a quicker change by allowing the change even if the
//     `candidate_delay` is not the most likely one according to the histogram.
//  2. ヒット数が kMinRequiredHits 以上で、ヒストグラムが最小値を超えている。
//  3. フィルタ長を超える遅延差の場合は遷移を早める。
// 詳細はコメント内コード参照。
//
// 入力:
//  - candidate_delay     : 検証対象の遅延。
//
// 戻り値:
//  - is_histogram_valid  : 1 なら候補が有効、0 なら無効。
static int HistogramBasedValidation(const BinaryDelayEstimator* self,
                                    int candidate_delay) {
  float fraction = 1.f;
  float histogram_threshold = self->histogram[self->compare_delay];
  const int delay_difference = candidate_delay - self->last_delay;
  int is_histogram_valid = 0;

  // ヒストグラム判定では last_delay の値に係数 fraction を掛けた閾値と
  // 候補ビンを比較する。fraction は候補との差 (delay_difference) に応じて線形に変化し、
  // 因果性維持やフィルタ長の制約を加味して遷移を早める場合がある。
  // さらにヒット数が十分（kMinRequiredHits 以上）かつヒストグラムが最小閾値
  // (`kMinHistogramThreshold`) を超える必要がある。

  // 候補との差に応じた比較値 histogram_threshold を計算。

  if (delay_difference > self->allowed_offset) {
    fraction = 1.f - kFractionSlope * (delay_difference - self->allowed_offset);
    fraction = (fraction > kMinFractionWhenPossiblyCausal
                    ? fraction
                    : kMinFractionWhenPossiblyCausal);
  } else if (delay_difference < 0) {
    fraction =
        kMinFractionWhenPossiblyNonCausal - kFractionSlope * delay_difference;
    fraction = (fraction > 1.f ? 1.f : fraction);
  }
  histogram_threshold *= fraction;
  histogram_threshold =
      (histogram_threshold > kMinHistogramThreshold ? histogram_threshold
                                                    : kMinHistogramThreshold);

  is_histogram_valid =
      (self->histogram[candidate_delay] >= histogram_threshold) &&
      (self->candidate_hits > kMinRequiredHits);

  return is_histogram_valid;
}

// ProcessBinarySpectrum() で求めた `candidate_delay` をロバストに検証する。
// ヒストグラム検証結果とロバスト統計を組み合わせる。
// `is_instantaneous_valid` and the `is_histogram_valid` and combines them
// 呼び出し前に HistogramBasedValidation() を実行しておく必要がある。
// 組み合わせ手順は下記コメントを参照。
//
// 入力:
//  - candidate_delay         : 検証対象の遅延。
//  - is_instantaneous_valid  : ProcessBinarySpectrum() での即時判定。
//  - is_histogram_valid      : ヒストグラムによる判定。
//
// 戻り値:
//  - is_robust               : 1 なら候補が信頼できる。0 ならそれ以外。
static int RobustValidation(const BinaryDelayEstimator* self,
                            int candidate_delay,
                            int is_instantaneous_valid,
                            int is_histogram_valid) {
  int is_robust = 0;

  // 最終判定は (1) ヒストグラム検証 と (2) ロバスト統計 の結果に基づく。
  // `is_instantaneous_valid` and 2) the histogram based with result stored in
  // `is_histogram_valid`.
  //   i) Before we actually have a valid estimate (`last_delay` == -2), we say
  //      a candidate is valid if either algorithm states so
  //      (`is_instantaneous_valid` OR `is_histogram_valid`).
  is_robust =
      (self->last_delay < 0) && (is_instantaneous_valid || is_histogram_valid);
  //  ii) Otherwise, we need both algorithms to be certain
  //      (`is_instantaneous_valid` AND `is_histogram_valid`)
  is_robust |= is_instantaneous_valid && is_histogram_valid;
  // ヒストグラム検証が優先される例外もある。
  //      the instantaneous one if `is_histogram_valid` = 1 and the histogram
  //      is significantly strong.
  is_robust |= is_histogram_valid &&
               (self->histogram[candidate_delay] > self->last_delay_histogram);

  return is_robust;
}

void InitBinaryDelayEstimatorFarend(BinaryDelayEstimatorFarend* self) {
  memset(self->binary_far_history, 0, sizeof(self->binary_far_history));
  memset(self->far_bit_counts, 0, sizeof(self->far_bit_counts));
}



void AddBinaryFarSpectrum(BinaryDelayEstimatorFarend* handle,
                                 uint32_t binary_far_spectrum) {
  // バイナリスペクトル履歴をシフトし、現在の `binary_far_spectrum` を追加。
  memmove(&(handle->binary_far_history[1]), &(handle->binary_far_history[0]),
          (MAX_DELAY - 1) * sizeof(uint32_t));
  handle->binary_far_history[0] = binary_far_spectrum;

  // 遠端バイナリスペクトルのビット数履歴をシフトし、現在のビット数を追加。
  // 
  memmove(&(handle->far_bit_counts[1]), &(handle->far_bit_counts[0]),
          (MAX_DELAY - 1) * sizeof(int));
  handle->far_bit_counts[0] = BitCount(binary_far_spectrum);
}
void InitBinaryDelayEstimator(BinaryDelayEstimator* self) {
  memset(self->bit_counts, 0, sizeof(self->bit_counts));
  memset(self->binary_near_history, 0, sizeof(self->binary_near_history));
  for (int i = 0; i <= MAX_DELAY; ++i) {
    self->mean_bit_counts[i] = (20 << 9);  // 20 in Q9.
    self->histogram[i] = 0.f;
  }
  self->minimum_probability = kMaxBitCountsQ9;          // 32 in Q9.
  self->last_delay_probability = (int)kMaxBitCountsQ9;  // 32 in Q9.

  // 推定不能時の既定値。エラーには -1 を返す。
  self->last_delay = -2;

  self->last_candidate_delay = -2;
  self->compare_delay = MAX_DELAY;
  self->candidate_hits = 0;
  self->last_delay_histogram = 0.f;

  self->allowed_offset = 0;
}



int ProcessBinarySpectrum(BinaryDelayEstimator* self,
                                 uint32_t binary_near_spectrum) {
  int candidate_delay = -1;
  int valid_candidate = 0;
  int hist_valid_dbg = 0;  // 0/1: histogram validity

  int32_t value_best_candidate = kMaxBitCountsQ9;
  int32_t value_worst_candidate = 0;
  int32_t valley_depth = 0;

  
  // lookahead=0 固定のため履歴操作なし
  self->binary_near_history[0] = binary_near_spectrum;

  // 遅延ごとのスペクトルと比較し、`bit_counts` に格納。
  BitCountComparison(binary_near_spectrum, self->farend->binary_far_history,
                     MAX_DELAY, self->bit_counts);

  // `bit_counts` を平滑化した `mean_bit_counts` を更新。
  for (int i = 0; i < MAX_DELAY; i++) {
    // `bit_counts` is constrained to [0, 32], meaning we can smooth with a
    // 係数は最大 2^26。Q9 表現を使用。
    int32_t bit_count = (self->bit_counts[i] << 9);  // Q9 表現。

    // 遠端信号が十分に存在するときのみ `mean_bit_counts` を更新。
    // `far_bit_counts` が 0 なら遠端は弱く、エコー条件が悪いとみなす。
    // その場合は更新しない。
    if (self->farend->far_bit_counts[i] > 0) {
      // 右シフト量を `far_bit_counts` に応じた区分線形で調整。
      int shifts = kShiftsAtZero;
      shifts -= (kShiftsLinearSlope * self->farend->far_bit_counts[i]) >> 4;
      MeanEstimator(bit_count, shifts, &(self->mean_bit_counts[i]));
    }
  }

  // `candidate_delay` と良/悪候補の値を求める。
  // 
  for (int i = 0; i < MAX_DELAY; i++) {
    if (self->mean_bit_counts[i] < value_best_candidate) {
      value_best_candidate = self->mean_bit_counts[i];
      candidate_delay = i;
    }
    if (self->mean_bit_counts[i] > value_worst_candidate) {
      value_worst_candidate = self->mean_bit_counts[i];
    }
  }
  valley_depth = value_worst_candidate - value_best_candidate;

  // `value_best_candidate` は一致確率を示す指標。
  // `candidate_delay` being an accurate delay (a small `value_best_candidate`
  // 以下の処理で `last_delay` を更新するか判断する。
  // 
  // 1) If the difference bit counts between the best and the worst delay
  //    candidates is too small we consider the situation to be unreliable and
  //    don't update `last_delay`.
  // 2) If the situation is reliable we update `last_delay` if the value of the
  //    best candidate delay has a value less than
  //     i) an adaptive threshold `minimum_probability`, or
  //    ii) this corresponding value `last_delay_probability`, but updated at
  //        this time instant.

  // `minimum_probability` を更新。
  if ((self->minimum_probability > kProbabilityLowerLimit) &&
      (valley_depth > kProbabilityMinSpread)) {
    // ハード閾値は Q9 で 17 未満にならないようにする。
    // 曲線の谷が十分深い（良候補と悪候補の差が大きい）ことも条件。
    // 
    // 
    int32_t threshold = value_best_candidate + kProbabilityOffset;
    if (threshold < kProbabilityLowerLimit) {
      threshold = kProbabilityLowerLimit;
    }
    if (self->minimum_probability > threshold) {
      self->minimum_probability = threshold;
    }
  }
  // `last_delay_probability` を更新。
  // マルコフ型モデルで徐々にレベルを上げる。
  self->last_delay_probability++;
  // `candidate_delay` を検証し、信頼できる即時遅延が得られたか判断する。
  // 
  //  1) 谷が十分深いか (`valley_depth` > `kProbabilityOffset`)
  // 
  //  2) The depth of the valley is deep enough
  //      (`value_best_candidate` < `minimum_probability`)
  //     and deeper than the best estimate so far
  //      (`value_best_candidate` < `last_delay_probability`)
  valid_candidate = ((valley_depth > kProbabilityOffset) &&
                     ((value_best_candidate < self->minimum_probability) ||
                      (value_best_candidate < self->last_delay_probability)));

  // 遠端信号が非定常かを確認。
  const bool non_stationary_farend =
      std::any_of(self->farend->far_bit_counts,
                  self->farend->far_bit_counts + MAX_DELAY,
                  [](int a) { return a > 0; });

  if (non_stationary_farend) {
    // 遠端が非定常のときのみ統計を更新する（定常なら推定値が凍結されるため）。
    // 
    UpdateRobustValidationStatistics(self, candidate_delay, valley_depth,
                                     value_best_candidate);
  }

  {
    int is_histogram_valid = HistogramBasedValidation(self, candidate_delay);
    hist_valid_dbg = is_histogram_valid;
    valid_candidate = RobustValidation(self, candidate_delay, valid_candidate,
                                       is_histogram_valid);
  }

  // 遠端が非定常で、妥当な候補がある場合のみ遅延推定を更新。
  // 
  if (non_stationary_farend && valid_candidate) {
    if (candidate_delay != self->last_delay) {
      self->last_delay_histogram =
          (self->histogram[candidate_delay] > kLastHistogramMax
               ? kLastHistogramMax
               : self->histogram[candidate_delay]);
      // `last_delay` を変更したがヒストグラム的に最尤でなかった場合、
      // ヒストグラムを補正する。
      if (self->histogram[candidate_delay] <
          self->histogram[self->compare_delay]) {
        self->histogram[self->compare_delay] = self->histogram[candidate_delay];
      }
    }
    self->last_delay = candidate_delay;
    if (value_best_candidate < self->last_delay_probability) {
      self->last_delay_probability = value_best_candidate;
    }
    self->compare_delay = self->last_delay;
  }

  // 100 回ごとに候補やヒストグラム値などをデバッグ出力。
  // 
  {
    static int dbg_counter = 0;
    dbg_counter++;
    if (dbg_counter % 100 == 0) {
      float hist_val = 0.f;
      if (candidate_delay >= 0 && candidate_delay < MAX_DELAY) {
        hist_val = self->histogram[candidate_delay];
      }
      fprintf(stderr,
              "[DelayEstimator] block=%d cand=%d hist_val=%.3f hist_valid=%d last=%d\n",
              dbg_counter, candidate_delay, hist_val, hist_valid_dbg, self->last_delay);
    }
  }

  return self->last_delay;
}




void MeanEstimator(int32_t new_value,
                   int factor,
                   int32_t* mean_value) {
  int32_t diff = new_value - *mean_value;

  // mean_new = mean_value + ((new_value - mean_value) >> factor);
  if (diff < 0) {
    diff = -((-diff) >> factor);
  } else {
    diff = (diff >> factor);
  }
  *mean_value += diff;
}

 
