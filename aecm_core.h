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
// すべての Aecm_* API はポインタ無しで利用できます。

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
  int farBufWritePos;
  int farBufReadPos;
  int knownDelay;
  int lastKnownDelay;
  int firstVAD;  // Parameter to control poorly initialized channels

  // フレーム/ブロック一致のため、中間フレーム用リングバッファは不要

  int16_t farBuf[FAR_BUF_LEN];

  // mult は 16k 固定運用のため不要
  

  // Delay estimation variables（固定長値型）
  DelayEstimatorFarend delay_estimator_farend;
  DelayEstimator delay_estimator;
  // Far end history variables
  // TODO(bjornv): Replace `far_history` with ring_buffer.
  uint16_t far_history[PART_LEN1 * MAX_DELAY];
  int far_history_pos;

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
  int16_t channelStored[PART_LEN1];
  int16_t channelAdapt16[PART_LEN1];
  int32_t channelAdapt32[PART_LEN1];
  int16_t xBuf[PART_LEN2];       // farend
  int16_t dBufNoisy[PART_LEN2];  // nearend
  int16_t outBuf[PART_LEN];

  int32_t echoFilt[PART_LEN1];
  int16_t nearFilt[PART_LEN1];
  

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
int Aecm_InitCore();

// Create/Freeは不要（単一インスタンス、固定長バッファ）。


////////////////////////////////////////////////////////////////////////////////
// 既知のエコーパス形状でチャネルを初期化（g_aecm を対象）。
void Aecm_InitEchoPathCore(const int16_t* echo_path);

////////////////////////////////////////////////////////////////////////////////
// 1フレーム（=1ブロック）処理。g_aecm を用いる。
int Aecm_ProcessFrame(const int16_t* farend,
                      const int16_t* nearend,
                      int16_t* out);

////////////////////////////////////////////////////////////////////////////////
// 1ブロック処理。g_aecm を用いる。
int Aecm_ProcessBlock(const int16_t* farend,
                      const int16_t* nearend,
                      int16_t* out);

////////////////////////////////////////////////////////////////////////////////
// Farend フレーム（FRAME_LEN サンプル）を g_aecm 側のバッファへ投入。
void Aecm_BufferFarFrame(const int16_t* const farend);

////////////////////////////////////////////////////////////////////////////////
// 既知遅延を考慮して g_aecm 側バッファから Farend フレームを取得。
void Aecm_FetchFarFrame(int16_t* const farend, int knownDelay);

////////////////////////////////////////////////////////////////////////////////
// Moves the pointer to the next entry and inserts `far_spectrum` and
// corresponding Q-domain in its buffer（固定Q=0だが引数は互換のため維持）。
//
// Inputs:
//      - self          : Pointer to the delay estimation instance
//      - far_spectrum  : Pointer to the far end spectrum
//      - far_q         : Q-domain of far end spectrum（固定Q=0）
//
// Far スペクトル履歴（g_aecm）を更新（固定Q=0のため Q は保持しない）。
void Aecm_UpdateFarHistory(uint16_t* far_spectrum);

////////////////////////////////////////////////////////////////////////////////
// Returns a pointer to the far end spectrum aligned to current near end
// spectrum. The function DelayEstimatorProcess(...) should have been
// called before AlignedFarend(...). Otherwise, you get the pointer to the
// previous frame. The memory is only valid until the next call of
// DelayEstimatorProcess(...).
//
// Inputs:
//      - self              : Pointer to the AECM instance.
//      - delay             : Current delay estimate.
//
// Output:
//      - far_q             : The Q-domain of the aligned far end spectrum（固定Q=0）
//
// Return value:
//      - far_spectrum      : Pointer to the aligned far end spectrum
//                            NULL - Error
//
// 近端に整列済みの Far スペクトルを返す（g_aecm 内の履歴に基づく）。
// 固定Q=0のため、Q出力は行わない。
const uint16_t* Aecm_AlignedFarend(int delay);

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
int16_t Aecm_CalcSuppressionGain();

///////////////////////////////////////////////////////////////////////////////
// This function calculates the log of energies for nearend, farend and
// estimated echoes. There is also an update of energy decision levels,
// i.e. internal VAD.
//
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - far_spectrum      : Pointer to farend spectrum.
//      - far_q             : Q-domain of farend spectrum.
//      - nearEner          : Near end energy for current block（固定Q=0）。
//
// Output:
//     - echoEst            : Estimated echo in Q(xfa_q+RESOLUTION_CHANNEL16).
//
// 近端/遠端/推定エコーのエネルギーを計算し、VAD 閾値などを更新（g_aecm）。
void Aecm_CalcEnergies(const uint16_t* far_spectrum,
                       int16_t far_q,
                       uint32_t nearEner,
                       int32_t* echoEst);

///////////////////////////////////////////////////////////////////////////////
// This function calculates the step size used in channel estimation
// Inputs:
//      - aecm              : Pointer to the AECM instance.
// Return value:
//      - mu                : Stepsize in log2(), i.e. number of shifts.
// NLMS ステップサイズ（log2）を計算（g_aecm）。
int16_t Aecm_CalcStepSize();

///////////////////////////////////////////////////////////////////////////////
// This function performs channel estimation.
// NLMS and decision on channel storage.
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - far_spectrum      : Absolute value of the farend signal in Q(far_q)
//      - far_q             : Q-domain of the farend signal
//      - dfa               : Absolute value of the nearend signal（固定Q=0）
//      - mu                : NLMS step size.
// Input/Output:
//      - echoEst           : Estimated echo in Q(far_q+RESOLUTION_CHANNEL16).
// チャネル推定（NLMS）を実行し、保存/復元の判定も行う（g_aecm）。
void Aecm_UpdateChannel(const uint16_t* far_spectrum,
                        int16_t far_q,
                        const uint16_t* const dfa,
                        int16_t mu,
                        int32_t* echoEst);

 

///////////////////////////////////////////////////////////////////////////////
 

 

#endif
