/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Performs echo control (suppression) with FFT routines in fixed-point.
// グローバルのデフォルトインスタンス `g_aecm` を内部で参照し、
// すべての API はポインタ無しで利用できます。

#ifndef MODULES_AUDIO_PROCESSING_AECM_AECM_CORE_H_
#define MODULES_AUDIO_PROCESSING_AECM_AECM_CORE_H_

extern "C" {
#include "signal_processing_library.h"
#include "real_fft.h"
}
#include "aecm_defines.h"
#include "delay_estimator_internal.h"

 

#ifdef MSC_VER  // visual c++
#define ALIGN8_BEG __declspec(align(8))
#define ALIGN8_END
#else  // gcc or icc
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
  int firstVAD;  // Parameter to control poorly initialized channels

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
  int16_t xBuf[PART_LEN2];       // farend x[n]
  int16_t yBuf[PART_LEN2];       // nearend y[n]
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

  struct RealFFT real_fft;

} AecmCore;

// デフォルトの単一インスタンス（グローバル・シングルトン）。
extern AecmCore g_aecm;

////////////////////////////////////////////////////////////////////////////////
// デフォルトインスタンス g_aecm を初期化（再初期化可）。
// Return value: 0 成功, -1 失敗
int InitCore();

// Create/Freeは不要（単一インスタンス、固定長バッファ）。


////////////////////////////////////////////////////////////////////////////////
// 既知のエコーパス形状でチャネルを初期化（g_aecm を対象）。
void InitEchoPathCore(const int16_t* echo_path);

////////////////////////////////////////////////////////////////////////////////
// 1フレーム（=1ブロック）処理。g_aecm を用いる。
int ProcessFrame(const int16_t* x_frame,
                      const int16_t* y_frame,
                      int16_t* e_frame);

////////////////////////////////////////////////////////////////////////////////
// 1ブロック処理。g_aecm を用いる。
int ProcessBlock(const int16_t* x_block,
                      const int16_t* y_block,
                      int16_t* e_block);

////////////////////////////////////////////////////////////////////////////////
// 遠端参照信号フレーム（x[n], FRAME_LEN サンプル）を g_aecm 側のバッファへ投入。
void BufferFarFrame(const int16_t* const x_frame);

////////////////////////////////////////////////////////////////////////////////
// 既知遅延を考慮して g_aecm 側バッファから整列済みの遠端フレームを取得。
void FetchFarFrame(int16_t* const x_frame, int knownDelay);

////////////////////////////////////////////////////////////////////////////////
// Moves the pointer to the next entry and inserts `x_spectrum` in its buffer
// （内部Qは固定で0）。
//
// Inputs:
//      - self          : Pointer to the delay estimation instance
//      - x_spectrum    : Pointer to the far end spectrum
//
// Far スペクトル履歴（g_aecm）を更新（固定Q=0のため Q は保持しない）。
void UpdateFarHistory(uint16_t* x_spectrum);

////////////////////////////////////////////////////////////////////////////////
// Returns a pointer to the far end spectrum aligned to current near end
// spectrum. The function DelayEstimatorProcess(...) should have been
// called before AlignedFarX(...). Otherwise, you get the pointer to the
// previous frame. The memory is only valid until the next call of
// DelayEstimatorProcess(...).
//
// Inputs:
//      - self              : Pointer to the AECM instance.
//      - delay             : Current delay estimate.
//
// Return value:
//      - x_spectrum        : Pointer to the aligned far end spectrum
//                            NULL - Error
//
// 近端に整列済みの Far スペクトルを返す（g_aecm 内の履歴に基づく）。
// 固定Q=0のため、Q出力は行わない。
const uint16_t* AlignedFarX(int delay);

///////////////////////////////////////////////////////////////////////////////
// This function calculates the suppression gain that is used in the
// Wiener filter.
//
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//
// Return value:
//      - supGain           : Suppression gain with which to scale the noise
//                            level (Q14).
//
// 抑圧ゲイン（NLP）を算出（g_aecm を参照）。
int16_t CalcSuppressionGain();

///////////////////////////////////////////////////////////////////////////////
// This function calculates the log of energies for nearend, farend and
// estimated echoes. There is also an update of energy decision levels,
// i.e. internal VAD.
//
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - X_mag             : Pointer to farend magnitude spectrum.
//      - Y_energy          : Near end energy for current block（固定Q=0）。
//
// Output:
//     - S_mag              : Estimated echo in Q(RESOLUTION_CHANNEL16)。
//
// 近端/遠端/推定エコーのエネルギーを計算し、VAD 閾値などを更新（g_aecm）。
void CalcEnergies(const uint16_t* X_mag,
                       uint32_t Y_energy,
                       int32_t* S_mag);

///////////////////////////////////////////////////////////////////////////////
// This function calculates the step size used in channel estimation
// Inputs:
//      - aecm              : Pointer to the AECM instance.
// Return value:
//      - mu                : Stepsize in log2(), i.e. number of shifts.
// NLMS ステップサイズ（log2）を計算（g_aecm）。
int16_t CalcStepSize();

///////////////////////////////////////////////////////////////////////////////
// This function performs channel estimation.
// NLMS and decision on channel storage.
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - X_mag             : Absolute value of the farend signal（Q0）
//      - x_q               : Q-domain of the farend signal（常に0を指定）
//      - Y_mag             : Absolute value of the nearend signal（固定Q=0）
//      - mu                : NLMS step size.
// Input/Output:
//      - S_mag             : Estimated echo in Q(RESOLUTION_CHANNEL16)。
// チャネル推定（NLMS）を実行し、保存/復元の判定も行う（g_aecm）。
void UpdateChannel(const uint16_t* X_mag,
                        int16_t x_q,
                        const uint16_t* const Y_mag,
                        int16_t mu,
                        int32_t* S_mag);

 

///////////////////////////////////////////////////////////////////////////////
 

 

#endif
