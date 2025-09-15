/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Performs echo control (suppression) with fft routines in fixed-point.

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
  int far_q_domains[MAX_DELAY];

  // NLP は常時有効。フラグは不要。
  int16_t fixedDelay;

  uint32_t totCount;

  int16_t dfaCleanQDomain;
  int16_t dfaCleanQDomainOld;
  int16_t dfaNoisyQDomain;
  int16_t dfaNoisyQDomainOld;

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

////////////////////////////////////////////////////////////////////////////////
// This function initializes the AECM instant created with
// Aecm_CreateCore()
// Input:
//      - aecm          : Pointer to the AECM instance
//      - samplingFreq  : Sampling Frequency
// Output:
//      - aecm          : Initialized instance
// Return value         :  0 - Ok
//                        -1 - Error
int Aecm_InitCore(AecmCore* const aecm);

// Create/Freeは不要（単一インスタンス、固定長バッファ）。


////////////////////////////////////////////////////////////////////////////////
// This function resets the echo channel adaptation with the specified channel.
// Input:
//      - aecm          : Pointer to the AECM instance
//      - echo_path     : Pointer to the data that should initialize the echo
//                        path
// Output:
//      - aecm          : Initialized instance
void Aecm_InitEchoPathCore(AecmCore* aecm, const int16_t* echo_path);

////////////////////////////////////////////////////////////////////////////////
// This function processes frames and sends blocks to
// Inputs:
//      - aecm          : Pointer to the AECM instance
//      - farend        : In buffer containing one frame of echo signal
//      - nearendNoisy  : In buffer containing one frame of nearend+echo signal
//                        without NS
//      - nearendClean  : In buffer containing one frame of nearend+echo signal
//                        with NS
// Output:
//      - out           : Out buffer, one frame of nearend signal          :
int Aecm_ProcessFrame(AecmCore* aecm,
                            const int16_t* farend,
                            const int16_t* nearend,
                            int16_t* out);

////////////////////////////////////////////////////////////////////////////////
// This function is called for every block within one frame
// This function is called by Aecm_ProcessFrame(...)
// Inputs:
//      - aecm          : Pointer to the AECM instance
//      - farend        : In buffer containing one block of echo signal
//      - nearendNoisy  : In buffer containing one frame of nearend+echo signal
//                        without NS
//      - nearendClean  : In buffer containing one frame of nearend+echo signal
//                        with NS
// Output:
//      - out           : Out buffer, one block of nearend signal          :
int Aecm_ProcessBlock(AecmCore* aecm,
                            const int16_t* farend,
                            const int16_t* nearend,
                            int16_t* out);

////////////////////////////////////////////////////////////////////////////////
// Inserts a frame of data into farend buffer.
// Inputs:
//      - aecm          : Pointer to the AECM instance
//      - farend        : In buffer containing one frame of farend signal
//      - farLen        : Length of frame
void Aecm_BufferFarFrame(AecmCore* const aecm,
                               const int16_t* const farend);

////////////////////////////////////////////////////////////////////////////////
// Read the farend buffer to account for known delay
// Inputs:
//      - aecm          : Pointer to the AECM instance
//      - farend        : In buffer containing one frame of farend signal
//      - farLen        : Length of frame
//      - knownDelay    : known delay
void Aecm_FetchFarFrame(AecmCore* const aecm,
                              int16_t* const farend,
                              int knownDelay);

////////////////////////////////////////////////////////////////////////////////
// Moves the pointer to the next entry and inserts `far_spectrum` and
// corresponding Q-domain in its buffer.
//
// Inputs:
//      - self          : Pointer to the delay estimation instance
//      - far_spectrum  : Pointer to the far end spectrum
//      - far_q         : Q-domain of far end spectrum
//
void Aecm_UpdateFarHistory(AecmCore* self,
                                 uint16_t* far_spectrum,
                                 int far_q);

////////////////////////////////////////////////////////////////////////////////
// Returns a pointer to the far end spectrum aligned to current near end
// spectrum. The function DelayEstimatorProcessFix(...) should have been
// called before AlignedFarend(...). Otherwise, you get the pointer to the
// previous frame. The memory is only valid until the next call of
// DelayEstimatorProcessFix(...).
//
// Inputs:
//      - self              : Pointer to the AECM instance.
//      - delay             : Current delay estimate.
//
// Output:
//      - far_q             : The Q-domain of the aligned far end spectrum
//
// Return value:
//      - far_spectrum      : Pointer to the aligned far end spectrum
//                            NULL - Error
//
const uint16_t* Aecm_AlignedFarend(AecmCore* self, int* far_q, int delay);

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
int16_t Aecm_CalcSuppressionGain(AecmCore* const aecm);

///////////////////////////////////////////////////////////////////////////////
// This function calculates the log of energies for nearend, farend and
// estimated echoes. There is also an update of energy decision levels,
// i.e. internal VAD.
//
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - far_spectrum      : Pointer to farend spectrum.
//      - far_q             : Q-domain of farend spectrum.
//      - nearEner          : Near end energy for current block in
//                            Q(aecm->dfaQDomain).
//
// Output:
//     - echoEst            : Estimated echo in Q(xfa_q+RESOLUTION_CHANNEL16).
//
void Aecm_CalcEnergies(AecmCore* aecm,
                             const uint16_t* far_spectrum,
                             int16_t far_q,
                             uint32_t nearEner,
                             int32_t* echoEst);

///////////////////////////////////////////////////////////////////////////////
// This function calculates the step size used in channel estimation
// Inputs:
//      - aecm              : Pointer to the AECM instance.
// Return value:
//      - mu                : Stepsize in log2(), i.e. number of shifts.
int16_t Aecm_CalcStepSize(AecmCore* const aecm);

///////////////////////////////////////////////////////////////////////////////
// This function performs channel estimation.
// NLMS and decision on channel storage.
// Inputs:
//      - aecm              : Pointer to the AECM instance.
//      - far_spectrum      : Absolute value of the farend signal in Q(far_q)
//      - far_q             : Q-domain of the farend signal
//      - dfa               : Absolute value of the nearend signal
//                            (Q[aecm->dfaQDomain])
//      - mu                : NLMS step size.
// Input/Output:
//      - echoEst           : Estimated echo in Q(far_q+RESOLUTION_CHANNEL16).
void Aecm_UpdateChannel(AecmCore* aecm,
                              const uint16_t* far_spectrum,
                              int16_t far_q,
                              const uint16_t* const dfa,
                              int16_t mu,
                              int32_t* echoEst);

 

///////////////////////////////////////////////////////////////////////////////
 

 

#endif
