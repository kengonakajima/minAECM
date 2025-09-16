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

extern "C" {
#include "ring_buffer.h"
#include "signal_processing_library.h"
#include "aecm_defines.h"
}
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
// Target suppression levels for nlp modes
// log{0.001, 0.00001, 0.00000001}
static const int kInitCheck = 42;

typedef struct {
  int sampFreq;
  short bufSizeStart;
  int knownDelay;

  // Stores the last frame added to the farend buffer
  short farendOld[FRAME_LEN];
  short initFlag;  // indicates if AEC has been initialized

  // Variables used for averaging far end buffer size
  short counter;
  short sum;
  short firstVal;
  short checkBufSizeCtr;

  // Variables used for delay shifts
  short filtDelay;
  int timeForDelayChange;
  int ECstartup;
  int checkBuffSize;
  int delayChange;
  short lastDelayDiff;

  int16_t echoMode;

  // Structures
  RingBuffer farendBuf;
  int16_t farendBufData[kBufSizeSamp];

} AecMobile;

 

// Estimates delay to set the position of the farend buffer read pointer
// (controlled by knownDelay)
static int Aecm_EstBufDelay();

// Stuffs the farend buffer if the estimated delay is too large
static int Aecm_DelayComp();

// 単一インスタンス実体（アプリ側ラッパの状態）
static AecMobile am;

int32_t Aecm_Init() {
  AecmConfig aecConfig;

  // サブ構造を初期化
  memset(&am, 0, sizeof(am));
  InitBufferWith(&am.farendBuf, am.farendBufData,
                 kBufSizeSamp, sizeof(int16_t));

  // 16 kHz 固定
  am.sampFreq = 16000;

  // Initialize AECM core
  if (Aecm_InitCore() == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  // Initialize farend buffer
  InitBuffer(&am.farendBuf);

  am.initFlag = kInitCheck;  // indicates that initialization has been done

  am.delayChange = 1;

  am.sum = 0;
  am.counter = 0;
  am.checkBuffSize = 1;
  am.firstVal = 0;

  am.ECstartup = 1;
  am.bufSizeStart = 0;
  am.checkBufSizeCtr = 0;
  am.filtDelay = 0;
  am.timeForDelayChange = 0;
  am.knownDelay = 0;
  am.lastDelayDiff = 0;

  memset(&am.farendOld, 0, sizeof(am.farendOld));

  // Default settings
  aecConfig.echoMode = 3;

  if (Aecm_set_config(aecConfig) == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  return 0;
}

// Returns any error that is caused when buffering the
// farend signal.
int32_t Aecm_BufferFarend(const int16_t* farend) {
  // 最小構成: 簡易チェックのみ
  if (farend == NULL) return AECM_NULL_POINTER_ERROR;
  if (am.initFlag != kInitCheck) return AECM_UNINITIALIZED_ERROR;

  // TODO(unknown): Is this really a good idea?
  if (!am.ECstartup) {
    Aecm_DelayComp();
  }

  WriteBuffer(&am.farendBuf, farend, FRAME_LEN);

  return 0;
}

int32_t Aecm_Process(const int16_t* nearend,
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
    // The AECM is in the start up mode
    // AECM is disabled until the soundcard buffer and farend buffers are OK

    // Mechanism to ensure that the soundcard buffer is reasonably stable.
    if (am.checkBuffSize) {
      am.checkBufSizeCtr++;
      // Before we fill up the far end buffer we require the amount of data on
      // the sound card to be stable (+/-8 ms) compared to the first value. This
      // comparison is made during the following 4 consecutive frames. If it
      // seems to be stable then we start to fill up the far end buffer.

      if (am.counter == 0) {
        am.firstVal = kFixedMsInSndCardBuf;
        am.sum = 0;
      }

      if (abs(am.firstVal - kFixedMsInSndCardBuf) <
          MAX(0.2 * kFixedMsInSndCardBuf, 8)) { // しきい値は従来のNB値(8ms)相当で据え置き
        am.sum += kFixedMsInSndCardBuf;
        am.counter++;
      } else {
        am.counter = 0;
      }

      if (am.counter >= 6) {
        // The farend buffer size is determined in blocks of FRAME_LEN samples
        // Use 75% of the average value of the soundcard buffer
        // 16k: 1msあたり16サンプル, FRAME_LEN=64 -> 0.75 * 平均ms * 16 / 64
        am.bufSizeStart = MIN(
            (3 * am.sum) / (am.counter * 16),
            BUF_SIZE_FRAMES);
        // buffersize has now been determined
        am.checkBuffSize = 0;
      }

      if (am.checkBufSizeCtr > 50) {
        // for really bad sound cards, don't disable echocanceller for more than
        // 0.5 sec
        am.bufSizeStart = MIN(
            (3 * kFixedMsInSndCardBuf) / 16,
            BUF_SIZE_FRAMES);
        am.checkBuffSize = 0;
      }
    }

    // if checkBuffSize changed in the if-statement above
    if (!am.checkBuffSize) {
      // soundcard buffer is now reasonably stable
      // When the far end buffer is filled with approximately the same amount of
      // data as the amount on the sound card we end the start up phase and
      // start to cancel echoes.

      if (nmbrOfFilledBuffers == am.bufSizeStart) {
        am.ECstartup = 0;  // Enable the AECM
      } else if (nmbrOfFilledBuffers > am.bufSizeStart) {
        MoveReadPtr(&am.farendBuf,
                           (int)available_read(&am.farendBuf) -
                               (int)am.bufSizeStart * FRAME_LEN);
        am.ECstartup = 0;
      }
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
        Aecm_EstBufDelay();
      }

      // Call the AECM
      /*Aecm_ProcessFrame(farend, &nearend[FRAME_LEN * i],
       &out[FRAME_LEN * i]);*/
      if (Aecm_ProcessFrame(
              farend_ptr, &nearend[FRAME_LEN * i],
              &out[FRAME_LEN * i]) == -1)
        return -1;
    }
  }

  

  return 0;
}

int32_t Aecm_set_config(AecmConfig config) {

  if (am.initFlag != kInitCheck) {
    return AECM_UNINITIALIZED_ERROR;
  }

  if (config.echoMode < 0 || config.echoMode > 4) {
    return AECM_BAD_PARAMETER_ERROR;
  }
  am.echoMode = config.echoMode;

  if (am.echoMode == 0) {
    g_aecm.supGain = SUPGAIN_DEFAULT >> 3;
    g_aecm.supGainOld = SUPGAIN_DEFAULT >> 3;
    g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 3;
    g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 3;
    g_aecm.supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 3) - (SUPGAIN_ERROR_PARAM_B >> 3);
    g_aecm.supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 3) - (SUPGAIN_ERROR_PARAM_D >> 3);
  } else if (am.echoMode == 1) {
    g_aecm.supGain = SUPGAIN_DEFAULT >> 2;
    g_aecm.supGainOld = SUPGAIN_DEFAULT >> 2;
    g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 2;
    g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 2;
    g_aecm.supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 2) - (SUPGAIN_ERROR_PARAM_B >> 2);
    g_aecm.supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 2) - (SUPGAIN_ERROR_PARAM_D >> 2);
  } else if (am.echoMode == 2) {
    g_aecm.supGain = SUPGAIN_DEFAULT >> 1;
    g_aecm.supGainOld = SUPGAIN_DEFAULT >> 1;
    g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 1;
    g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 1;
    g_aecm.supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 1) - (SUPGAIN_ERROR_PARAM_B >> 1);
    g_aecm.supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 1) - (SUPGAIN_ERROR_PARAM_D >> 1);
  } else if (am.echoMode == 3) {
    g_aecm.supGain = SUPGAIN_DEFAULT;
    g_aecm.supGainOld = SUPGAIN_DEFAULT;
    g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A;
    g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D;
    g_aecm.supGainErrParamDiffAB =
        SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
    g_aecm.supGainErrParamDiffBD =
        SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;
  } else if (am.echoMode == 4) {
    g_aecm.supGain = SUPGAIN_DEFAULT << 1;
    g_aecm.supGainOld = SUPGAIN_DEFAULT << 1;
    g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A << 1;
    g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D << 1;
    g_aecm.supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A << 1) - (SUPGAIN_ERROR_PARAM_B << 1);
    g_aecm.supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B << 1) - (SUPGAIN_ERROR_PARAM_D << 1);
  }

  return 0;
}



static int Aecm_EstBufDelay() {
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

static int Aecm_DelayComp() {
  int nSampFar = (int)available_read(&am.farendBuf);
  const int maxStuffSamp = 10 * FRAME_LEN;

  int nSampSndCard = kFixedMsInSndCardBuf * kSamplesPerMs16k;
  int delayNew = nSampSndCard - nSampFar;

  if (delayNew > FAR_BUF_LEN - FRAME_LEN * 2) {
    // The difference of the buffer sizes is larger than the maximum
    // allowed known delay. Compensate by stuffing the buffer.
    int nSampAdd =
        (int)(MAX(((nSampSndCard >> 1) - nSampFar), FRAME_LEN));
    nSampAdd = MIN(nSampAdd, maxStuffSamp);

    MoveReadPtr(&am.farendBuf, -nSampAdd);
    am.delayChange = 1;  // the delay needs to be updated
  }

  return 0;
}

 
