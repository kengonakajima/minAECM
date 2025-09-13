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
// Target suppression levels for nlp modes
// log{0.001, 0.00001, 0.00000001}
static const int kInitCheck = 42;

typedef struct {
  int sampFreq;
  short bufSizeStart;
  int knownDelay;

  // Stores the last frame added to the farend buffer
  short farendOld[2][FRAME_LEN];
  short initFlag;  // indicates if AEC has been initialized

  // Variables used for averaging far end buffer size
  short counter;
  short sum;
  short firstVal;
  short checkBufSizeCtr;

  // Variables used for delay shifts
  short msInSndCardBuf;
  short filtDelay;
  int timeForDelayChange;
  int ECstartup;
  int checkBuffSize;
  int delayChange;
  short lastDelayDiff;

  int16_t echoMode;

  // Structures
  RingBuffer* farendBuf;

  AecmCore* aecmCore;
} AecMobile;

 

// Estimates delay to set the position of the farend buffer read pointer
// (controlled by knownDelay)
static int Aecm_EstBufDelay(AecMobile* aecm, short msInSndCardBuf);

// Stuffs the farend buffer if the estimated delay is too large
static int Aecm_DelayComp(AecMobile* aecm);

void* Aecm_Create() {
  // Allocate zero-filled memory.
  AecMobile* aecm = static_cast<AecMobile*>(calloc(1, sizeof(AecMobile)));

  aecm->aecmCore = Aecm_CreateCore();
  if (!aecm->aecmCore) {
    Aecm_Free(aecm);
    return NULL;
  }

  aecm->farendBuf = CreateBuffer(kBufSizeSamp, sizeof(int16_t));
  if (!aecm->farendBuf) {
    Aecm_Free(aecm);
    return NULL;
  }

  return aecm;
}

void Aecm_Free(void* aecmInst) {
  AecMobile* aecm = static_cast<AecMobile*>(aecmInst);

  if (aecm == NULL) {
    return;
  }

  Aecm_FreeCore(aecm->aecmCore);
  FreeBuffer(aecm->farendBuf);
  free(aecm);
}

int32_t Aecm_Init(void* aecmInst) {
  AecMobile* aecm = static_cast<AecMobile*>(aecmInst);
  AecmConfig aecConfig;

  if (aecm == NULL) {
    return -1;
  }

  // 16 kHz 固定
  aecm->sampFreq = 16000;

  // Initialize AECM core
  if (Aecm_InitCore(aecm->aecmCore) == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  // Initialize farend buffer
  InitBuffer(aecm->farendBuf);

  aecm->initFlag = kInitCheck;  // indicates that initialization has been done

  aecm->delayChange = 1;

  aecm->sum = 0;
  aecm->counter = 0;
  aecm->checkBuffSize = 1;
  aecm->firstVal = 0;

  aecm->ECstartup = 1;
  aecm->bufSizeStart = 0;
  aecm->checkBufSizeCtr = 0;
  aecm->filtDelay = 0;
  aecm->timeForDelayChange = 0;
  aecm->knownDelay = 0;
  aecm->lastDelayDiff = 0;

  memset(&aecm->farendOld, 0, sizeof(aecm->farendOld));

  // Default settings
  aecConfig.echoMode = 3;

  if (Aecm_set_config(aecm, aecConfig) == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  return 0;
}

// Returns any error that is caused when buffering the
// farend signal.
int32_t Aecm_BufferFarend(void* aecmInst,
                                const int16_t* farend) {
  AecMobile* aecm = static_cast<AecMobile*>(aecmInst);
  // 最小構成: 簡易チェックのみ
  if (aecm == NULL) return -1;
  if (farend == NULL) return AECM_NULL_POINTER_ERROR;
  if (aecm->initFlag != kInitCheck) return AECM_UNINITIALIZED_ERROR;

  // TODO(unknown): Is this really a good idea?
  if (!aecm->ECstartup) {
    Aecm_DelayComp(aecm);
  }

  WriteBuffer(aecm->farendBuf, farend, 160);

  return 0;
}

int32_t Aecm_Process(void* aecmInst,
                           const int16_t* nearend,
                           int16_t* out,
                           int16_t msInSndCardBuf) {
  AecMobile* aecm = static_cast<AecMobile*>(aecmInst);
  int32_t retVal = 0;
  size_t i;
  short nmbrOfFilledBuffers;
  size_t nFrames;
  

  if (aecm == NULL) {
    return -1;
  }

  if (nearend == NULL) {
    return AECM_NULL_POINTER_ERROR;
  }

  if (out == NULL) {
    return AECM_NULL_POINTER_ERROR;
  }

  if (aecm->initFlag != kInitCheck) {
    return AECM_UNINITIALIZED_ERROR;
  }

  // 16kHz固定のため、160サンプル固定
  const size_t nrOfSamples = 160;

  if (msInSndCardBuf < 0) {
    msInSndCardBuf = 0;
    retVal = AECM_BAD_PARAMETER_WARNING;
  } else if (msInSndCardBuf > 500) {
    msInSndCardBuf = 500;
    retVal = AECM_BAD_PARAMETER_WARNING;
  }
  msInSndCardBuf += 10;
  aecm->msInSndCardBuf = msInSndCardBuf;

  nFrames = nrOfSamples / FRAME_LEN; // 160/80=2（16kHz固定）

  if (aecm->ECstartup) {
    if (out != nearend) {
      memcpy(out, nearend, sizeof(short) * nrOfSamples);
    }

    nmbrOfFilledBuffers =
        (short)available_read(aecm->farendBuf) / FRAME_LEN;
    // The AECM is in the start up mode
    // AECM is disabled until the soundcard buffer and farend buffers are OK

    // Mechanism to ensure that the soundcard buffer is reasonably stable.
    if (aecm->checkBuffSize) {
      aecm->checkBufSizeCtr++;
      // Before we fill up the far end buffer we require the amount of data on
      // the sound card to be stable (+/-8 ms) compared to the first value. This
      // comparison is made during the following 4 consecutive frames. If it
      // seems to be stable then we start to fill up the far end buffer.

      if (aecm->counter == 0) {
        aecm->firstVal = aecm->msInSndCardBuf;
        aecm->sum = 0;
      }

      if (abs(aecm->firstVal - aecm->msInSndCardBuf) <
          MAX(0.2 * aecm->msInSndCardBuf, 8)) { // しきい値は従来のNB値(8ms)相当で据え置き
        aecm->sum += aecm->msInSndCardBuf;
        aecm->counter++;
      } else {
        aecm->counter = 0;
      }

      if (aecm->counter >= 6) {
        // The farend buffer size is determined in blocks of 80 samples
        // Use 75% of the average value of the soundcard buffer
        // 16k: 10msは80サンプル×2ブロック -> 0.75 * 平均ms * 2 / 10ms
        aecm->bufSizeStart = MIN(
            (3 * aecm->sum * 2) / (aecm->counter * 40),
            BUF_SIZE_FRAMES);
        // buffersize has now been determined
        aecm->checkBuffSize = 0;
      }

      if (aecm->checkBufSizeCtr > 50) {
        // for really bad sound cards, don't disable echocanceller for more than
        // 0.5 sec
        aecm->bufSizeStart = MIN(
            (3 * aecm->msInSndCardBuf * 2) / 40,
            BUF_SIZE_FRAMES);
        aecm->checkBuffSize = 0;
      }
    }

    // if checkBuffSize changed in the if-statement above
    if (!aecm->checkBuffSize) {
      // soundcard buffer is now reasonably stable
      // When the far end buffer is filled with approximately the same amount of
      // data as the amount on the sound card we end the start up phase and
      // start to cancel echoes.

      if (nmbrOfFilledBuffers == aecm->bufSizeStart) {
        aecm->ECstartup = 0;  // Enable the AECM
      } else if (nmbrOfFilledBuffers > aecm->bufSizeStart) {
        MoveReadPtr(aecm->farendBuf,
                           (int)available_read(aecm->farendBuf) -
                               (int)aecm->bufSizeStart * FRAME_LEN);
        aecm->ECstartup = 0;
      }
    }

  } else {
    // AECM is enabled

    // 16kHz: 10msは80サンプル×2ブロック
    for (i = 0; i < nFrames; i++) {
      int16_t farend[FRAME_LEN];
      const int16_t* farend_ptr = NULL;

      nmbrOfFilledBuffers =
          (short)available_read(aecm->farendBuf) / FRAME_LEN;

      // Check that there is data in the far end buffer
      if (nmbrOfFilledBuffers > 0) {
        // Get the next 80 samples from the farend buffer
        ReadBuffer(aecm->farendBuf, (void**)&farend_ptr, farend,
                          FRAME_LEN);

        // Always store the last frame for use when we run out of data
        memcpy(&(aecm->farendOld[i][0]), farend_ptr, FRAME_LEN * sizeof(short));
      } else {
        // We have no data so we use the last played frame
        memcpy(farend, &(aecm->farendOld[i][0]), FRAME_LEN * sizeof(short));
        farend_ptr = farend;
      }

      // 10ms（2ブロック）取り出し終わりのタイミングで1回だけ遅延推定
      if (i == nFrames - 1) {
        Aecm_EstBufDelay(aecm, aecm->msInSndCardBuf);
      }

      // Call the AECM
      /*Aecm_ProcessFrame(aecm->aecmCore, farend, &nearend[FRAME_LEN * i],
       &out[FRAME_LEN * i], aecm->knownDelay);*/
      if (Aecm_ProcessFrame(
              aecm->aecmCore, farend_ptr, &nearend[FRAME_LEN * i],
              &out[FRAME_LEN * i]) == -1)
        return -1;
    }
  }

  

  return retVal;
}

int32_t Aecm_set_config(void* aecmInst, AecmConfig config) {
  AecMobile* aecm = static_cast<AecMobile*>(aecmInst);

  if (aecm == NULL) {
    return -1;
  }

  if (aecm->initFlag != kInitCheck) {
    return AECM_UNINITIALIZED_ERROR;
  }


  if (config.echoMode < 0 || config.echoMode > 4) {
    return AECM_BAD_PARAMETER_ERROR;
  }
  aecm->echoMode = config.echoMode;

  if (aecm->echoMode == 0) {
    aecm->aecmCore->supGain = SUPGAIN_DEFAULT >> 3;
    aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT >> 3;
    aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 3;
    aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 3;
    aecm->aecmCore->supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 3) - (SUPGAIN_ERROR_PARAM_B >> 3);
    aecm->aecmCore->supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 3) - (SUPGAIN_ERROR_PARAM_D >> 3);
  } else if (aecm->echoMode == 1) {
    aecm->aecmCore->supGain = SUPGAIN_DEFAULT >> 2;
    aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT >> 2;
    aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 2;
    aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 2;
    aecm->aecmCore->supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 2) - (SUPGAIN_ERROR_PARAM_B >> 2);
    aecm->aecmCore->supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 2) - (SUPGAIN_ERROR_PARAM_D >> 2);
  } else if (aecm->echoMode == 2) {
    aecm->aecmCore->supGain = SUPGAIN_DEFAULT >> 1;
    aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT >> 1;
    aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A >> 1;
    aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D >> 1;
    aecm->aecmCore->supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A >> 1) - (SUPGAIN_ERROR_PARAM_B >> 1);
    aecm->aecmCore->supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B >> 1) - (SUPGAIN_ERROR_PARAM_D >> 1);
  } else if (aecm->echoMode == 3) {
    aecm->aecmCore->supGain = SUPGAIN_DEFAULT;
    aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT;
    aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A;
    aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D;
    aecm->aecmCore->supGainErrParamDiffAB =
        SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
    aecm->aecmCore->supGainErrParamDiffBD =
        SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;
  } else if (aecm->echoMode == 4) {
    aecm->aecmCore->supGain = SUPGAIN_DEFAULT << 1;
    aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT << 1;
    aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A << 1;
    aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D << 1;
    aecm->aecmCore->supGainErrParamDiffAB =
        (SUPGAIN_ERROR_PARAM_A << 1) - (SUPGAIN_ERROR_PARAM_B << 1);
    aecm->aecmCore->supGainErrParamDiffBD =
        (SUPGAIN_ERROR_PARAM_B << 1) - (SUPGAIN_ERROR_PARAM_D << 1);
  }

  return 0;
}



static int Aecm_EstBufDelay(AecMobile* aecm, short msInSndCardBuf) {
  short delayNew, nSampSndCard;
  short nSampFar = (short)available_read(aecm->farendBuf);
  short diff;

  nSampSndCard = msInSndCardBuf * kSamplesPerMs16k;

  delayNew = nSampSndCard - nSampFar;

  if (delayNew < FRAME_LEN) {
    MoveReadPtr(aecm->farendBuf, FRAME_LEN);
    delayNew += FRAME_LEN;
  }

  aecm->filtDelay =
      MAX(0, (8 * aecm->filtDelay + 2 * delayNew) / 10);

  diff = aecm->filtDelay - aecm->knownDelay;
  if (diff > 224) {
    if (aecm->lastDelayDiff < 96) {
      aecm->timeForDelayChange = 0;
    } else {
      aecm->timeForDelayChange++;
    }
  } else if (diff < 96 && aecm->knownDelay > 0) {
    if (aecm->lastDelayDiff > 224) {
      aecm->timeForDelayChange = 0;
    } else {
      aecm->timeForDelayChange++;
    }
  } else {
    aecm->timeForDelayChange = 0;
  }
  aecm->lastDelayDiff = diff;

  if (aecm->timeForDelayChange > 25) {
    aecm->knownDelay = MAX((int)aecm->filtDelay - 160, 0);
  }
  return 0;
}

static int Aecm_DelayComp(AecMobile* aecm) {
  int nSampFar = (int)available_read(aecm->farendBuf);
  int nSampSndCard, delayNew, nSampAdd;
  const int maxStuffSamp = 10 * FRAME_LEN;

  nSampSndCard = aecm->msInSndCardBuf * kSamplesPerMs16k;
  delayNew = nSampSndCard - nSampFar;

  if (delayNew > FAR_BUF_LEN - FRAME_LEN * 2) {
    // The difference of the buffer sizes is larger than the maximum
    // allowed known delay. Compensate by stuffing the buffer.
    nSampAdd =
        (int)(MAX(((nSampSndCard >> 1) - nSampFar), FRAME_LEN));
    nSampAdd = MIN(nSampAdd, maxStuffSamp);

    MoveReadPtr(aecm->farendBuf, -nSampAdd);
    aecm->delayChange = 1;  // the delay needs to be updated
  }

  return 0;
}

 
