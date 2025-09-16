/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "echo_control_mobile.h"

#ifdef AEC_DEBUG
#include <stdio.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "ring_buffer.h"
#include "signal_processing_library.h"
#include "aecm_defines.h"
#include "aecm_core.h"

 
 

#define BUF_SIZE_FRAMES 50  // buffer size (frames)
// Maximum length of resampled signal. Must be an integer multiple of frames
// (ceil(1/(1 + MIN_SKEW)*2) + 1)*FRAME_LEN
// The factor of 2 handles wb, and the + 1 is as a safety margin

static const size_t kBufSizeSamp =
    BUF_SIZE_FRAMES * FRAME_LEN;  // buffer size (samples)
// 16kHz固定のため、1msあたり16サンプル
static const int kSamplesPerMs16k = 16;
// 本実装では、デバイス入出力の往復遅延を 50ms とし、
// 従来の +10ms マージン込みで固定 60ms とする。
static const short kFixedMsInSndCardBuf = 60;
static const short kStartupFramesRaw =
    (short)((kFixedMsInSndCardBuf * kSamplesPerMs16k) / FRAME_LEN);
static const short kStartupFrames =
    (kStartupFramesRaw <= 0)
        ? 1
        : (kStartupFramesRaw > BUF_SIZE_FRAMES ? BUF_SIZE_FRAMES
                                               : kStartupFramesRaw);
// Target suppression levels for nlp modes
// log{0.001, 0.00001, 0.00000001}
static const int kInitCheck = 42;

typedef struct {
  short bufSizeStart;
  int knownDelay;

  // Stores the last frame added to the farend buffer
  short farendOld[FRAME_LEN];
  short initFlag;  // indicates if AEC has been initialized

  // Variables used for delay shifts
  short filtDelay;
  int timeForDelayChange;
  int ECstartup;
  short lastDelayDiff;

  // Structures
  RingBuffer farendBuf;
  int16_t farendBufData[kBufSizeSamp];

} AecMobile;

 

// Estimates delay to set the position of the farend buffer read pointer
// (controlled by knownDelay)
static int EstimateBufDelay();

// 単一インスタンス実体（アプリ側ラッパの状態）
static AecMobile am;

int32_t Init() {
  AecmConfig aecConfig;

  // サブ構造を初期化
  memset(&am, 0, sizeof(am));
  InitBufferWith(&am.farendBuf, am.farendBufData,
                 kBufSizeSamp, sizeof(int16_t));

  // Initialize AECM core
  if (InitCore() == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  // Initialize farend buffer
  InitBuffer(&am.farendBuf);

  am.initFlag = kInitCheck;  // indicates that initialization has been done
  am.bufSizeStart = kStartupFrames;
  am.ECstartup = 1;
  am.filtDelay = 0;
  am.timeForDelayChange = 0;
  am.knownDelay = 0;
  am.lastDelayDiff = 0;

  memset(&am.farendOld, 0, sizeof(am.farendOld));

  // Default settings
  aecConfig.echoMode = 3;

  if (SetConfig(aecConfig) == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  return 0;
}

// Returns any error that is caused when buffering the
// farend signal.
int32_t BufferFarend(const int16_t* farend) {
  // 最小構成: 簡易チェックのみ
  if (farend == NULL) return AECM_NULL_POINTER_ERROR;
  if (am.initFlag != kInitCheck) return AECM_UNINITIALIZED_ERROR;

  WriteBuffer(&am.farendBuf, farend, FRAME_LEN);

  return 0;
}

int32_t Process(const int16_t* nearend,
                           int16_t* out) {
  // ループ変数や一時値は使用直前に宣言
  
  if (nearend == NULL) {
    return AECM_NULL_POINTER_ERROR;
  }

  if (out == NULL) {
    return AECM_NULL_POINTER_ERROR;
  }

  if (am.initFlag != kInitCheck) {
    return AECM_UNINITIALIZED_ERROR;
  }

  // 16kHz固定。I/Oブロック=FRAME_LEN（FRAME_LEN=PART_LEN=64 -> 64サンプル）
  const size_t nrOfSamples = FRAME_LEN;

  // サウンドカードバッファ遅延は固定

  const size_t nFrames = nrOfSamples / FRAME_LEN; // 64/64=1（16kHz固定）

  if (am.ECstartup) {
    if (out != nearend) {
      memcpy(out, nearend, sizeof(short) * nrOfSamples);
    }

    short nmbrOfFilledBuffers =
        (short)available_read(&am.farendBuf) / FRAME_LEN;
    if (nmbrOfFilledBuffers >= am.bufSizeStart) {
      if (nmbrOfFilledBuffers > am.bufSizeStart) {
        MoveReadPtr(&am.farendBuf,
                    (int)available_read(&am.farendBuf) -
                        (int)am.bufSizeStart * FRAME_LEN);
      }
      am.ECstartup = 0;
    }

  } else {
    // AECM is enabled

    // 1フレーム=1ブロック（FRAME_LEN=64）
    for (size_t i = 0; i < nFrames; i++) {
      int16_t farend[FRAME_LEN];
      const int16_t* farend_ptr = NULL;

      short nmbrOfFilledBuffers =
          (short)available_read(&am.farendBuf) / FRAME_LEN;

      // Check that there is data in the far end buffer
      if (nmbrOfFilledBuffers > 0) {
        // Get the next FRAME_LEN samples from the farend buffer
        ReadBuffer(&am.farendBuf, (void**)&farend_ptr, farend,
                          FRAME_LEN);

        // Always store the last frame for use when we run out of data
        memcpy(am.farendOld, farend_ptr, FRAME_LEN * sizeof(short));
      } else {
        // We have no data so we use the last played frame
        memcpy(farend, am.farendOld, FRAME_LEN * sizeof(short));
        farend_ptr = farend;
      }

      // フレーム終端で1回だけ遅延推定
      if (i == nFrames - 1) {
        EstimateBufDelay();
      }

      // Call the AECM
      /*ProcessFrame(farend, &nearend[FRAME_LEN * i],
       &out[FRAME_LEN * i]);*/
      if (ProcessFrame(
              farend_ptr, &nearend[FRAME_LEN * i],
              &out[FRAME_LEN * i]) == -1)
        return -1;
    }
  }

  

  return 0;
}

int32_t SetConfig(AecmConfig config) {
  if (am.initFlag != kInitCheck) {
    return AECM_UNINITIALIZED_ERROR;
  }

  if (config.echoMode != 3) {
    return AECM_BAD_PARAMETER_ERROR;
  }

  g_aecm.supGain = SUPGAIN_DEFAULT;
  g_aecm.supGainOld = SUPGAIN_DEFAULT;
  g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A;
  g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D;
  g_aecm.supGainErrParamDiffAB =
      SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
  g_aecm.supGainErrParamDiffBD =
      SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;

  return 0;
}



static int EstimateBufDelay() {
  short nSampFar = (short)available_read(&am.farendBuf);
  short nSampSndCard = kFixedMsInSndCardBuf * kSamplesPerMs16k;
  short delayNew = nSampSndCard - nSampFar;

  if (delayNew < FRAME_LEN) {
    MoveReadPtr(&am.farendBuf, FRAME_LEN);
    delayNew += FRAME_LEN;
  }

  am.filtDelay =
      MAX(0, (8 * am.filtDelay + 2 * delayNew) / 10);

  short diff = am.filtDelay - am.knownDelay;
  if (diff > 224) {
    if (am.lastDelayDiff < 96) {
      am.timeForDelayChange = 0;
    } else {
      am.timeForDelayChange++;
    }
  } else if (diff < 96 && am.knownDelay > 0) {
    if (am.lastDelayDiff > 224) {
      am.timeForDelayChange = 0;
    } else {
      am.timeForDelayChange++;
    }
  } else {
    am.timeForDelayChange = 0;
  }
  am.lastDelayDiff = diff;

  if (am.timeForDelayChange > 25) {
    am.knownDelay = MAX((int)am.filtDelay - (2 * FRAME_LEN), 0);
  }
  return 0;
}
