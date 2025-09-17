// 固定小数点 FFT ルーチンによるエコー抑圧を実装。
// グローバル既定インスタンス `g_aecm` を内部参照し、
// すべての API をポインタ無しで利用できる。

#include "util.h"
#include "aecm_defines.h"
#include "delay_estimator_internal.h"

 

#ifdef MSC_VER  // Visual C++
#define ALIGN8_BEG __declspec(align(8))
#define ALIGN8_END
#else  // gcc または icc
#define ALIGN8_BEG
#define ALIGN8_END __attribute__((aligned(8)))
#endif

typedef struct {
  int16_t real;
  int16_t imag;
} ComplexInt16;

typedef struct {
  int xBufWritePos;
  int xBufReadPos;
  int knownDelay;
  int lastKnownDelay;
  int firstVAD;  // 初期化が不十分なチャネルを制御するためのフラグ

  // フレーム/ブロック一致のため、中間フレーム用リングバッファは不要

  int16_t xFrameBuf[FAR_BUF_LEN];

  // mult は 16k 固定運用のため不要
  

  // Delay estimation variables（固定長値型）
  DelayEstimatorFarend delay_estimator_farend;
  DelayEstimator delay_estimator;
  // Far end history variables
  uint16_t xHistory[PART_LEN1 * MAX_DELAY];
  int xHistoryPos;

  uint32_t totCount;

  // Q ドメインは教育用に固定（0）で運用。
  int16_t dfaCleanQDomain;     // 常に 0
  int16_t dfaCleanQDomainOld;  // 常に 0（差分も 0）
  int16_t dfaNoisyQDomain;     // 常に 0
  int16_t dfaNoisyQDomainOld;  // 常に 0

  int16_t nearLogEnergy[MAX_BUF_LEN];
  int16_t farLogEnergy;
  int16_t echoAdaptLogEnergy[MAX_BUF_LEN];
  int16_t echoStoredLogEnergy[MAX_BUF_LEN];

  // バッファは素直な配列として保持（NEON用のアラインメントは不要）
  int16_t hStored[PART_LEN1];
  int16_t hAdapt16[PART_LEN1];
  int32_t hAdapt32[PART_LEN1];
  int16_t xBuf[PART_LEN2];       // 遠端信号 x[n]
  int16_t yBuf[PART_LEN2];       // 近端信号 y[n]
  int16_t eOverlapBuf[PART_LEN];

  int32_t sMagSmooth[PART_LEN1];
  int16_t yMagSmooth[PART_LEN1];
  

  int32_t mseAdaptOld;
  int32_t mseStoredOld;
  int32_t mseThreshold;

  int16_t farEnergyMin;
  int16_t farEnergyMax;
  int16_t farEnergyMaxMin;
  int16_t farEnergyVAD;
  int16_t farEnergyMSE;
  int currentVADValue;
  int16_t vadUpdateCount;

  int16_t startupState;
  int16_t mseChannelCount;
  int16_t supGain;
  int16_t supGainOld;

  int16_t supGainErrParamA;
  int16_t supGainErrParamD;
  int16_t supGainErrParamDiffAB;
  int16_t supGainErrParamDiffBD;

} AecmCore;

// デフォルトの単一インスタンス（グローバル・シングルトン）。
extern AecmCore g_aecm;

////////////////////////////////////////////////////////////////////////////////
// AECMを初期化
int InitCore();

// 既知のエコーパス形状でチャネルを初期化（g_aecm を対象）。
void InitEchoPathCore(const int16_t* echo_path);

// 1フレーム（=1ブロック）処理
int ProcessFrame(const int16_t* x_frame, const int16_t* y_frame, int16_t* e_frame);

// 1ブロック処理
int ProcessBlock(const int16_t* x_block, const int16_t* y_block, int16_t* e_block);

// デバッグ/調整用: 抑圧ステージをバイパスするフラグを設定。
void AecmCoreSetBypassWiener(int enable);
void AecmCoreSetBypassNlp(int enable);


// 遠端参照信号フレーム（x[n], FRAME_LEN サンプル）を g_aecm 側のバッファへ投入。
void BufferFarFrame(const int16_t* const x_frame);

// 既知遅延を考慮して g_aecm 側バッファから整列済みの遠端フレームを取得。
void FetchFarFrame(int16_t* const x_frame, int knownDelay);

// g_aecm の遠端スペクトル履歴を更新（Q=0 固定のため Q は保持しない）。
void UpdateFarHistory(uint16_t* x_spectrum);

// 近端に整列済みの Far スペクトルを返す（g_aecm 内の履歴に基づく）。 固定Q=0のため、Q出力は行わない。
const uint16_t* AlignedFarX(int delay);


// Wiener フィルタで使用する抑圧ゲインを計算する。  抑圧ゲイン（NLP）を算出.
int16_t CalcSuppressionGain();

// 近端/遠端/推定エコーのエネルギーを計算し、VAD 閾値などを更新（g_aecm）。
void CalcEnergies(const uint16_t* X_mag, uint32_t Y_energy, int32_t* S_mag);


// チャネル推定で使用するステップサイズを計算する。
// NLMS ステップサイズ（log2）を計算
int16_t CalcStepSize();

// 入力:
//      - X_mag             : 遠端信号の絶対値スペクトル（Q0）
//      - x_q               : 遠端信号の Q ドメイン（常に 0）
//      - Y_mag             : 近端信号の絶対値スペクトル（Q0）
//      - mu                : NLMS ステップサイズ
// 入出力:
//      - S_mag             : 推定エコー（Q=RESOLUTION_CHANNEL16）
// チャネル推定（NLMS）を実行し、保存/復元の判定も行う（g_aecm）。
void UpdateChannel(const uint16_t* X_mag, int16_t x_q, const uint16_t* const Y_mag, int16_t mu, int32_t* S_mag);

 
