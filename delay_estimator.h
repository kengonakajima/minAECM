#include <stdint.h>
#include "aecm_defines.h"

 

static const int32_t kMaxBitCountsQ9 = (32 << 9);  // Q9表現での一致ビット数（最大32）。

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
  int last_candidate_delay;
  int compare_delay;
  int candidate_hits;
  float histogram[MAX_DELAY + 1];
  float last_delay_histogram;

  // 遠端履歴（外部 Farend 構造体へのポインタ）。
  BinaryDelayEstimatorFarend* farend;
} BinaryDelayEstimator;

typedef int32_t SpectrumType;

typedef struct {
  SpectrumType mean_far_spectrum[PART_LEN1];
  int far_spectrum_initialized;

  BinaryDelayEstimatorFarend binary_farend;
} DelayEstimatorFarend;

typedef struct {
  SpectrumType mean_near_spectrum[PART_LEN1];
  int near_spectrum_initialized;

  BinaryDelayEstimator binary_handle;
} DelayEstimator;

// 動的確保APIは削除（固定長・単一インスタンス前提）。

// 遅延推定器の遠端状態を初期化する。
void InitBinaryDelayEstimatorFarend();



// 遠端の2値スペクトルを内部履歴バッファへ追加する。
void AddBinaryFarSpectrum(uint32_t binary_far_spectrum);


// 近端側の2値遅延推定器状態を初期化する。
void InitBinaryDelayEstimator();



// 近端の2値スペクトルを処理し、現在の遅延推定値を返す。
// 戻り値:
//    - delay                 :  0以上  - 計算された遅延値。
//                              -2    - 推定に十分なデータがない。
int ProcessBinarySpectrum(uint32_t binary_near_spectrum);


// 遠端側の単一インスタンス遅延推定器状態を初期化する。
void InitDelayEstimatorFarend();
// 近端側の単一インスタンス遅延推定器状態を初期化する。
void InitDelayEstimator();
// 最新の近端スペクトルを処理し、推定された遅延を返す。
int DelayEstimatorProcess(const uint16_t* near_spectrum, const uint16_t* far_spectrum);

 
