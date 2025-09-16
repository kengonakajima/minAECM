#include "echo_control_mobile.h"

#ifdef AEC_DEBUG
#include <stdio.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "aecm_core.h"

 
 

#define BUF_SIZE_FRAMES 50  // バッファ長（フレーム数単位）
// 再サンプル後信号の最大長（フレーム整合のため整数倍とする）。
// 16kHz 固定のため (ceil(1/(1 + MIN_SKEW)*2) + 1)*FRAME_LEN に相当。
// 係数2は広帯域対応、+1は安全マージン。

static const size_t kBufSizeSamp =
    BUF_SIZE_FRAMES * FRAME_LEN;  // バッファ長（サンプル数単位）
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

  // 最終投入された遠端フレームを保持
  short farendOld[FRAME_LEN];
  short initFlag;  // AECM が初期化済みかどうか

  // 遅延調整に用いる変数
  short filtDelay;
  int timeForDelayChange;
  int ECstartup;
  short lastDelayDiff;

  // バッファ構造体
  RingBuffer farendBuf;
  int16_t farendBufData[kBufSizeSamp];

} AecMobile;

 

// 遠端バッファの既知遅延に基づく読み出し位置を推定
static int EstimateBufDelay();

// 単一インスタンス実体（アプリ側ラッパの状態）
static AecMobile am;

int32_t Init() {
  AecmConfig aecConfig;

  // サブ構造を初期化
  memset(&am, 0, sizeof(am));
  InitBufferWith(&am.farendBuf, am.farendBufData,
                 kBufSizeSamp, sizeof(int16_t));

  // AECM コアを初期化
  if (InitCore() == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  // 遠端バッファを初期化
  InitBuffer(&am.farendBuf);

  am.initFlag = kInitCheck;  // 初期化済みであることを示す
  am.bufSizeStart = kStartupFrames;
  am.ECstartup = 1;
  am.filtDelay = 0;
  am.timeForDelayChange = 0;
  am.knownDelay = 0;
  am.lastDelayDiff = 0;

  memset(&am.farendOld, 0, sizeof(am.farendOld));

  // 既定設定
  aecConfig.echoMode = 3;

  if (SetConfig(aecConfig) == -1) {
    return AECM_UNSPECIFIED_ERROR;
  }

  return 0;
}

// 遠端信号をバッファリングする際のエラーコードを返す。
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

  // サウンドカード遅延は固定値
  const size_t nFrames = nrOfSamples / FRAME_LEN;  // 64/64=1（16kHz固定）

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
    // AECM 有効時の処理

    // 1フレーム=1ブロック（FRAME_LEN=64）
    for (size_t i = 0; i < nFrames; i++) {
      int16_t farend[FRAME_LEN];
      const int16_t* farend_ptr = NULL;

      short nmbrOfFilledBuffers =
          (short)available_read(&am.farendBuf) / FRAME_LEN;

      // 遠端バッファにデータがあるか確認
      if (nmbrOfFilledBuffers > 0) {
        // 次の FRAME_LEN 分を遠端バッファから取得
        ReadBuffer(&am.farendBuf, (void**)&farend_ptr, farend,
                          FRAME_LEN);

        // データ枯渇時に備え直近フレームを保存
        memcpy(am.farendOld, farend_ptr, FRAME_LEN * sizeof(short));
      } else {
        // データが無ければ最後のフレームを再利用
        memcpy(farend, am.farendOld, FRAME_LEN * sizeof(short));
        farend_ptr = farend;
      }

      // フレーム終端で1回だけ遅延推定
      if (i == nFrames - 1) {
        EstimateBufDelay();
      }

      // AECM 本体を呼び出す
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
