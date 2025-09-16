#include "aecm_core.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "echo_control_mobile.h"
#include "delay_estimator_wrapper.h"

// デフォルトの単一インスタンス（ポインタ省略呼び出し用）
AecmCore g_aecm;

 
 



// 16 kHz 用エコーチャネルの初期化テーブル
static const int16_t kChannelStored16kHz[PART_LEN1] = {
    2040, 1590, 1405, 1385, 1451, 1562, 1726, 1882, 1953, 2010, 2040,
    2027, 2014, 1980, 1869, 1732, 1635, 1572, 1517, 1444, 1367, 1294,
    1245, 1233, 1260, 1303, 1373, 1441, 1499, 1549, 1582, 1621, 1676,
    1741, 1802, 1861, 1921, 1983, 2040, 2102, 2170, 2265, 2375, 2515,
    2651, 2781, 2922, 3075, 3253, 3471, 3738, 3976, 4151, 4258, 4308,
    4288, 4270, 4253, 4237, 4179, 4086, 3947, 3757, 3484, 3153};

 



// 次のエントリへポインタを進め、バッファに `x_spectrum` を格納
// （Qは固定0）。
//
// 入力:
//      - self          : Pointer to the delay estimation instance
//      - x_spectrum    : Pointer to the far end spectrum
//
void UpdateFarHistory(uint16_t* x_spectrum) {
  // 新しいバッファ位置を算出
  g_aecm.xHistoryPos++;
  if (g_aecm.xHistoryPos >= MAX_DELAY) {
    g_aecm.xHistoryPos = 0;
  }
  // Q-domain は固定Q=0のため保持不要
  // 遠端スペクトル用バッファを更新
  memcpy(&(g_aecm.xHistory[g_aecm.xHistoryPos * PART_LEN1]), x_spectrum,
         sizeof(uint16_t) * PART_LEN1);
}

// 現在の近端に整列した遠端スペクトルのポインタを返す
// DelayEstimatorProcess(...) を事前に呼び出していることが前提。
// そうでない場合は前回フレームのポインタが返る。
// メモリは次に DelayEstimatorProcess(...) を呼ぶまで有効。
// 
//
// 入力:
//      - self              : Pointer to the AECM instance.
//      - delay             : Current delay estimate.
// 戻り値:
//      - x_spectrum        : Pointer to the aligned far end spectrum
//                            NULL - Error
//
const uint16_t* AlignedFarX(int delay) {
  int buffer_position = 0;
  // 元実装では DCHECK で検査していたが、最小構成では省略。
  buffer_position = g_aecm.xHistoryPos - delay;

  // バッファ位置を正規化
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  // 整列済みの遠端スペクトルを返す
  return &(g_aecm.xHistory[buffer_position * PART_LEN1]);
}

// Create/Freeは廃止。InitCore()で内部状態を初期化する。

void InitEchoPathCore(const int16_t* echo_path) {
  // 保存チャネルをリセット
  memcpy(g_aecm.hStored, echo_path, sizeof(int16_t) * PART_LEN1);
  // 適応チャネルをリセット
  memcpy(g_aecm.hAdapt16, echo_path, sizeof(int16_t) * PART_LEN1);
  for (int i = 0; i < PART_LEN1; i++) {
    g_aecm.hAdapt32[i] = (int32_t)g_aecm.hAdapt16[i] << 16;
  }

  // チャネル保存に関する変数を初期化
  g_aecm.mseAdaptOld = 1000;
  g_aecm.mseStoredOld = 1000;
  g_aecm.mseThreshold = WORD32_MAX;
  g_aecm.mseChannelCount = 0;
}

void CalcLinearEnergiesC(const uint16_t* X_mag,
                         int32_t* S_mag,
                         uint32_t* X_energy,
                         uint32_t* S_energy_adapt,
                         uint32_t* S_energy_stored) {
  // 遅延後の遠端信号と推定エコーのエネルギーを取得（保存/適応チャネル双方）
  // 
  for (int i = 0; i < PART_LEN1; i++) {
    S_mag[i] = MUL_16_U16(g_aecm.hStored[i], X_mag[i]);
    (*X_energy) += (uint32_t)(X_mag[i]);
    *S_energy_adapt += g_aecm.hAdapt16[i] * X_mag[i];
    (*S_energy_stored) += (uint32_t)S_mag[i];
  }
}

void StoreAdaptiveChannelC(const uint16_t* X_mag,
                           int32_t* S_mag) {
  // 起動中は毎ブロック保存チャネルを更新
  memcpy(g_aecm.hStored, g_aecm.hAdapt16, sizeof(int16_t) * PART_LEN1);
  // 推定エコーを再計算
  for (int i = 0; i < PART_LEN; i += 4) {
    S_mag[i] = MUL_16_U16(g_aecm.hStored[i], X_mag[i]);
    S_mag[i + 1] = MUL_16_U16(g_aecm.hStored[i + 1], X_mag[i + 1]);
    S_mag[i + 2] = MUL_16_U16(g_aecm.hStored[i + 2], X_mag[i + 2]);
    S_mag[i + 3] = MUL_16_U16(g_aecm.hStored[i + 3], X_mag[i + 3]);
  }
  // PART_LEN1 は PART_LEN + 1
  S_mag[PART_LEN] = MUL_16_U16(g_aecm.hStored[PART_LEN], X_mag[PART_LEN]);
}

void ResetAdaptiveChannelC() {
  // 連続 2 回、保存チャネルの MSE が適応チャネルより十分小さい場合、
  // 適応チャネルをリセットする。
  memcpy(g_aecm.hAdapt16, g_aecm.hStored,
         sizeof(int16_t) * PART_LEN1);
  // 32bit チャネル表現を復元
  for (int i = 0; i < PART_LEN; i += 4) {
    g_aecm.hAdapt32[i] = (int32_t)g_aecm.hStored[i] << 16;
    g_aecm.hAdapt32[i + 1] = (int32_t)g_aecm.hStored[i + 1] << 16;
    g_aecm.hAdapt32[i + 2] = (int32_t)g_aecm.hStored[i + 2] << 16;
    g_aecm.hAdapt32[i + 3] = (int32_t)g_aecm.hStored[i + 3] << 16;
  }
  g_aecm.hAdapt32[PART_LEN] = (int32_t)g_aecm.hStored[PART_LEN] << 16;
}



// AECM インスタンスを初期化する関数
// 入力:
//      - aecm            : Pointer to the Echo Suppression instance
//      - samplingFreq   : Sampling Frequency
//
// 出力:
//      - aecm            : Initialized instance
//
// 戻り値               :  0 - 成功
//                        -1 - Error
//
int InitCoreImpl() {
  // 16kHz 固定

  g_aecm.xBufWritePos = 0;
  g_aecm.xBufReadPos = 0;
  g_aecm.knownDelay = 0;
  g_aecm.lastKnownDelay = 0;

  // FRAME_LEN=PART_LEN のため中間フレーム用リングバッファは不要

  memset(g_aecm.xBuf, 0, sizeof(g_aecm.xBuf));
  memset(g_aecm.yBuf, 0, sizeof(g_aecm.yBuf));
  memset(g_aecm.eOverlapBuf, 0, sizeof(g_aecm.eOverlapBuf));

  g_aecm.totCount = 0;

  if (InitDelayEstimatorFarend(&g_aecm.delay_estimator_farend) != 0) {
    return -1;
  }
  // 遠端側と遅延推定器を接続
  g_aecm.delay_estimator.farend_wrapper = &g_aecm.delay_estimator_farend;
  if (InitDelayEstimator(&g_aecm.delay_estimator) != 0) {
    return -1;
  }
  // 遠端履歴をゼロ初期化
  memset(g_aecm.xHistory, 0, sizeof(uint16_t) * PART_LEN1 * MAX_DELAY);
  g_aecm.xHistoryPos = MAX_DELAY;

  g_aecm.dfaCleanQDomain = 0;
  g_aecm.dfaCleanQDomainOld = 0;
  g_aecm.dfaNoisyQDomain = 0;
  g_aecm.dfaNoisyQDomainOld = 0;

  memset(g_aecm.nearLogEnergy, 0, sizeof(g_aecm.nearLogEnergy));
  g_aecm.farLogEnergy = 0;
  memset(g_aecm.echoAdaptLogEnergy, 0, sizeof(g_aecm.echoAdaptLogEnergy));
  memset(g_aecm.echoStoredLogEnergy, 0, sizeof(g_aecm.echoStoredLogEnergy));

  // エコーチャネルを既定形状（16 kHz 固定）で初期化
  InitEchoPathCore(kChannelStored16kHz);

  memset(g_aecm.sMagSmooth, 0, sizeof(g_aecm.sMagSmooth));
  memset(g_aecm.yMagSmooth, 0, sizeof(g_aecm.yMagSmooth));

  g_aecm.farEnergyMin = WORD16_MAX;
  g_aecm.farEnergyMax = WORD16_MIN;
  g_aecm.farEnergyMaxMin = 0;
  g_aecm.farEnergyVAD = FAR_ENERGY_MIN;  // 開始直後の誤検出（音声とみなさない）を防ぐ
                                        // 
  g_aecm.farEnergyMSE = 0;
  g_aecm.currentVADValue = 0;
  g_aecm.vadUpdateCount = 0;
  g_aecm.firstVAD = 1;

  g_aecm.startupState = 0;
  g_aecm.supGain = SUPGAIN_DEFAULT;
  g_aecm.supGainOld = SUPGAIN_DEFAULT;

  g_aecm.supGainErrParamA = SUPGAIN_ERROR_PARAM_A;
  g_aecm.supGainErrParamD = SUPGAIN_ERROR_PARAM_D;
  g_aecm.supGainErrParamDiffAB = SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
  g_aecm.supGainErrParamDiffBD = SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;

  // コンパイル時に前提条件を static_assert で確認
  // アセンブリ実装が依存するため、修正時は該当ファイルを要確認。
  static_assert(PART_LEN % 16 == 0, "PART_LEN is not a multiple of 16");

  // 実数 FFT の order を設定
  g_aecm.real_fft.order = PART_LEN_SHIFT;

  return 0;
}

// デフォルトインスタンス初期化
int InitCore() { return InitCoreImpl(); }

// Freeは不要

int ProcessFrame(const int16_t* x_frame,
                      const int16_t* y_frame,
                      int16_t* e_frame) {
  int16_t x_block[FRAME_LEN];

  // デフォルトインスタンスに対して Far をバッファし、既知遅延位置を取得
  BufferFarFrame(x_frame);
  FetchFarFrame(x_block, g_aecm.knownDelay);

  // FRAME_LEN と PART_LEN を一致させたため、1ブロックで直接処理
  if (ProcessBlock(x_block, y_frame, e_frame) == -1) {
    return -1;
  }
  return 0;
}

// 非対称フィルタ処理を行う。
//
// 入力:
//      - filtOld       : Previous filtered value.
//      - inVal         : New input value.
//      - stepSizePos   : Step size when we have a positive contribution.
//      - stepSizeNeg   : Step size when we have a negative contribution.
//
// 出力:
//
// 戻り値: フィルタ適用後の値。
//
int16_t AsymFilt(const int16_t filtOld,
                            const int16_t inVal,
                            const int16_t stepSizePos,
                            const int16_t stepSizeNeg) {
  int16_t retVal;

  if ((filtOld == WORD16_MAX) | (filtOld == WORD16_MIN)) {
    return inVal;
  }
  retVal = filtOld;
  if (filtOld > inVal) {
    retVal -= (filtOld - inVal) >> stepSizeNeg;
  } else {
    retVal += (inVal - filtOld) >> stepSizePos;
  }

  return retVal;
}

// ExtractFractionPart(a, zeros)
//
// `a` の仮数部を、先頭ゼロ数 `zeros` を加味した Q8 の int16_t として返す。
// ゼロ数との整合性チェックは行わない。
// 
int16_t ExtractFractionPart(uint32_t a, int zeros) {
  return (int16_t)(((a << zeros) & 0x7FFFFFFF) >> 23);
}

// `energy` の対数 (Q8) を計算して返す。入力は Q(`q_domain`) と想定。
// 入力の `energy` は Q(`q_domain`)（本縮約では q_domain=0）で与えられる想定。
int16_t LogOfEnergyInQ8(uint32_t energy, int q_domain) {
  static const int16_t kLogLowValue = PART_LEN_SHIFT << 7;
  int16_t log_energy_q8 = kLogLowValue;
  if (energy > 0) {
    int zeros = NormU32(energy);
    int16_t frac = ExtractFractionPart(energy, zeros);
    // `energy` の log2 を Q8 で取得。
    log_energy_q8 += ((31 - zeros) << 8) + frac - (q_domain << 8);
  }
  return log_energy_q8;
}

// 近端・遠端・推定エコーのエネルギーログを算出し、
// エネルギー閾値（内部 VAD）も更新する。
// 
//
//
// @param  aecm         [i/o]   Handle of the AECM instance.
// @param  X_mag        [in]    Pointer to farend spectrum magnitude.
// @param  Y_energy     [in]    Near end energy for current block in
//                              Q(aecm->dfaNoisyQDomain).
// @param  S_mag        [out]   Estimated echo in Q(xfa_q+RESOLUTION_CHANNEL16).
//
void CalcEnergies(const uint16_t* X_mag,
                       const uint32_t Y_energy,
                       int32_t* S_mag) {
  // ローカル変数
  uint32_t tmpAdapt = 0;
  uint32_t tmpStored = 0;
  uint32_t tmpFar = 0;

  int16_t tmp16;
  int16_t increase_max_shifts = 4;
  int16_t decrease_max_shifts = 11;
  int16_t increase_min_shifts = 11;
  int16_t decrease_min_shifts = 3;

  // 近端エネルギーの対数を求め、バッファへ格納

  // バッファをシフト
  memmove(g_aecm.nearLogEnergy + 1, g_aecm.nearLogEnergy,
          sizeof(int16_t) * (MAX_BUF_LEN - 1));

  // 近端振幅積分の対数 (nearEner)
  g_aecm.nearLogEnergy[0] = LogOfEnergyInQ8(Y_energy, g_aecm.dfaNoisyQDomain);

  CalcLinearEnergiesC(X_mag, S_mag, &tmpFar, &tmpAdapt, &tmpStored);

  // ログ履歴バッファをシフト
  memmove(g_aecm.echoAdaptLogEnergy + 1, g_aecm.echoAdaptLogEnergy,
          sizeof(int16_t) * (MAX_BUF_LEN - 1));
  memmove(g_aecm.echoStoredLogEnergy + 1, g_aecm.echoStoredLogEnergy,
          sizeof(int16_t) * (MAX_BUF_LEN - 1));

  // 遅延後遠端エネルギーの対数
  g_aecm.farLogEnergy = LogOfEnergyInQ8(tmpFar, 0);

  // 適応チャネル経由推定エコーの対数エネルギー
  g_aecm.echoAdaptLogEnergy[0] =
      LogOfEnergyInQ8(tmpAdapt, RESOLUTION_CHANNEL16);

  // 保存チャネル経由推定エコーの対数エネルギー
  g_aecm.echoStoredLogEnergy[0] =
      LogOfEnergyInQ8(tmpStored, RESOLUTION_CHANNEL16);

  // 遠端エネルギー関連の閾値（最小・最大・VAD・MSE）を更新
  if (g_aecm.farLogEnergy > FAR_ENERGY_MIN) {
    if (g_aecm.startupState == 0) {
      increase_max_shifts = 2;
      decrease_min_shifts = 2;
      increase_min_shifts = 8;
    }

    g_aecm.farEnergyMin =
        AsymFilt(g_aecm.farEnergyMin, g_aecm.farLogEnergy,
                            increase_min_shifts, decrease_min_shifts);
    g_aecm.farEnergyMax =
        AsymFilt(g_aecm.farEnergyMax, g_aecm.farLogEnergy,
                            increase_max_shifts, decrease_max_shifts);
    g_aecm.farEnergyMaxMin = (g_aecm.farEnergyMax - g_aecm.farEnergyMin);

    // 可変 VAD 領域サイズを算出
    tmp16 = 2560 - g_aecm.farEnergyMin;
    if (tmp16 > 0) {
      tmp16 = (int16_t)((tmp16 * FAR_ENERGY_VAD_REGION) >> 9);
    } else {
      tmp16 = 0;
    }
    tmp16 += FAR_ENERGY_VAD_REGION;

    if ((g_aecm.startupState == 0) | (g_aecm.vadUpdateCount > 1024)) {
      // 起動中または VAD 更新停止中
      g_aecm.farEnergyVAD = g_aecm.farEnergyMin + tmp16;
    } else {
      if (g_aecm.farEnergyVAD > g_aecm.farLogEnergy) {
        g_aecm.farEnergyVAD +=
            (g_aecm.farLogEnergy + tmp16 - g_aecm.farEnergyVAD) >> 6;
        g_aecm.vadUpdateCount = 0;
      } else {
        g_aecm.vadUpdateCount++;
      }
    }
    // MSE 閾値を VAD より少し高く設定
    g_aecm.farEnergyMSE = g_aecm.farEnergyVAD + (1 << 8);
  }

  // VAD 状態を更新
  if (g_aecm.farLogEnergy > g_aecm.farEnergyVAD) {
    if ((g_aecm.startupState == 0) | (g_aecm.farEnergyMaxMin > FAR_ENERGY_DIFF)) {
      // 起動中、または入力音声レベルのダイナミクスが大きい
      g_aecm.currentVADValue = 1;
    }
  } else {
    g_aecm.currentVADValue = 0;
  }
  if ((g_aecm.currentVADValue) && (g_aecm.firstVAD)) {
    g_aecm.firstVAD = 0;
    if (g_aecm.echoAdaptLogEnergy[0] > g_aecm.nearLogEnergy[0]) {
      // 推定エコーが近端信号より強い場合、
      // 初期値が大きすぎたと判断し、
      // チャネルを 1/8 にスケールダウンする。
      for (int i = 0; i < PART_LEN1; i++) {
        g_aecm.hAdapt16[i] >>= 3;
      }
      // 合わせて推定エコーのログ値も調整。
      g_aecm.echoAdaptLogEnergy[0] -= (3 << 8);
      g_aecm.firstVAD = 1;
    }
  }
}

// g_aecm の状態を基に NLMS のステップサイズ μ を決定する。
// 戻り値は log2 系（2^-μ）のシフト量として扱い、遠端エネルギーや
// スタートアップ状態に応じて自動調整される。
int16_t CalcStepSize() {
  int32_t tmp32;
  int16_t tmp16;
  int16_t mu = MU_MAX;

  // ここでは NLMS 型チャネル推定で用いるステップサイズ μ を計算
  // 
  if (!g_aecm.currentVADValue) {
    // 遠端エネルギーが低すぎる場合は更新しない
    mu = 0;
  } else if (g_aecm.startupState > 0) {
    if (g_aecm.farEnergyMin >= g_aecm.farEnergyMax) {
      mu = MU_MIN;
    } else {
      tmp16 = (g_aecm.farLogEnergy - g_aecm.farEnergyMin);
      tmp32 = tmp16 * MU_DIFF;
      tmp32 = DivW32W16(tmp32, g_aecm.farEnergyMaxMin);
      mu = MU_MIN - 1 - (int16_t)(tmp32);
      // -1 することで端数処理の代わりとし、若干大きめのステップにする
      // （NLMS での丸め誤差を補う目的）
    }
    if (mu < MU_MAX) {
      mu = MU_MAX;  // ステップサイズ 2^-MU_MAX に相当する上限
    }
  }

  return mu;
}

// NLMS によるチャネル推定と保存判定。
// X_mag: 遠端振幅スペクトル(Q0)、x_q: 遠端信号のQ(常に0)、
// Y_mag: 近端振幅スペクトル(Q0)、mu: CalcStepSize のシフト量、
// S_mag: 推定エコー（Q=RESOLUTION_CHANNEL16）。
void UpdateChannel(const uint16_t* X_mag,
                              const int16_t x_q,
                              const uint16_t* const Y_mag,
                              const int16_t mu,
                              int32_t* S_mag) {
  uint32_t tmpU32no1, tmpU32no2;
  int32_t tmp32no1, tmp32no2;
  int32_t mseStored;
  int32_t mseAdapt;

  int16_t zerosFar, zerosNum, zerosCh, zerosDfa;
  int16_t shiftChFar, shiftNum, shift2ResChan;
  int16_t tmp16no1;
  int16_t xfaQ, yMagQ;

  // NLMS ベースのチャネル推定で、
  // 上で計算した可変ステップ長を使用する。
  if (mu) {
    for (int i = 0; i < PART_LEN1; i++) {
      // オーバーフロー防止のためチャネルと遠端の正規化量を算出
      // 
      zerosCh = NormU32(g_aecm.hAdapt32[i]);
      zerosFar = NormU32((uint32_t)X_mag[i]);
      if (zerosCh + zerosFar > 31) {
        // 乗算しても安全な状態
        tmpU32no1 = UMUL_32_16(g_aecm.hAdapt32[i], X_mag[i]);
        shiftChFar = 0;
      } else {
        // 乗算前にシフトダウンが必要
        shiftChFar = 32 - zerosCh - zerosFar;
        // zerosCh==zerosFar==0 なら shiftChFar=32 となり、
        // 右シフト 32 は未定義なのでチェックする。
        // 
        {
          uint32_t shifted = (shiftChFar >= 32)
                                  ? 0u
                                  : (uint32_t)(g_aecm.hAdapt32[i] >> shiftChFar);
          tmpU32no1 = shifted * X_mag[i];
        }
      }
      // 分子の Q ドメインを決定
      zerosNum = NormU32(tmpU32no1);
      if (Y_mag[i]) {
        zerosDfa = NormU32((uint32_t)Y_mag[i]);
      } else {
        zerosDfa = 32;
      }
      tmp16no1 = zerosDfa - 2 + g_aecm.dfaNoisyQDomain - RESOLUTION_CHANNEL32 -
                 x_q + shiftChFar;
      if (zerosNum > tmp16no1 + 1) {
        xfaQ = tmp16no1;
        yMagQ = zerosDfa - 2;
      } else {
        xfaQ = zerosNum - 2;
        yMagQ = RESOLUTION_CHANNEL32 + x_q - g_aecm.dfaNoisyQDomain -
               shiftChFar + xfaQ;
      }
      // 同じ Q ドメインに揃えて加算
      tmpU32no1 = SHIFT_W32(tmpU32no1, xfaQ);
      tmpU32no2 = SHIFT_W32((uint32_t)Y_mag[i], yMagQ);
      tmp32no1 = (int32_t)tmpU32no2 - (int32_t)tmpU32no1;
      zerosNum = NormW32(tmp32no1);
      if ((tmp32no1) && (X_mag[i] > (CHANNEL_VAD << x_q))) {
        //
        // 更新が必要なケース
        //
        // 以下の計算を行いたい：
        //
        // tmp32no1 = Y_mag[i] - (aecm->channelAdapt[i] * X_mag[i])
        // tmp32norm = (i + 1)
        // aecm->channelAdapt[i] += (2^mu) * tmp32no1
        //                        / (tmp32norm * X_mag[i])
        //

        // 乗算でオーバーフローしないようにする。
        if (zerosNum + zerosFar > 31) {
          if (tmp32no1 > 0) {
            tmp32no2 =
                (int32_t)UMUL_32_16(tmp32no1, X_mag[i]);
          } else {
            tmp32no2 =
                -(int32_t)UMUL_32_16(-tmp32no1, X_mag[i]);
          }
          shiftNum = 0;
        } else {
          shiftNum = 32 - (zerosNum + zerosFar);
          if (tmp32no1 > 0) {
            tmp32no2 = (tmp32no1 >> shiftNum) * X_mag[i];
          } else {
            tmp32no2 = -((-tmp32no1 >> shiftNum) * X_mag[i]);
          }
        }
        // 周波数ビンに応じて正規化
        tmp32no2 = DivW32W16(tmp32no2, i + 1);
        // 適切な Q ドメインに揃える
        shift2ResChan =
            shiftNum + shiftChFar - xfaQ - mu - ((30 - zerosFar) << 1);
        if (NormW32(tmp32no2) < shift2ResChan) {
          tmp32no2 = WORD32_MAX;
        } else {
          tmp32no2 = SHIFT_W32(tmp32no2, shift2ResChan);
        }
        g_aecm.hAdapt32[i] = AddSatW32(g_aecm.hAdapt32[i], tmp32no2);
        if (g_aecm.hAdapt32[i] < 0) {
          // チャネル利得が負にならないよう強制
          g_aecm.hAdapt32[i] = 0;
        }
        g_aecm.hAdapt16[i] = (int16_t)(g_aecm.hAdapt32[i] >> 16);
      }
    }
  }
  // 適応チャネル更新ここまで


  // チャネルを保存するか復元するかを判定
  if ((g_aecm.startupState == 0) & (g_aecm.currentVADValue)) {
    // 起動中は毎ブロックチャネルを保存し、
    // 推定エコーも再計算する
    StoreAdaptiveChannelC(X_mag, S_mag);
  } else {
    if (g_aecm.farLogEnergy < g_aecm.farEnergyMSE) {
      g_aecm.mseChannelCount = 0;
    } else {
      g_aecm.mseChannelCount++;
    }
    // 検証に十分なデータがあれば、保存を検討
    if (g_aecm.mseChannelCount >= (MIN_MSE_COUNT + 10)) {
      // 十分なデータが揃った
      // 適応版と保存版の MSE を計算
      // 実際には平均絶対誤差に近い指標
      mseStored = 0;
      mseAdapt = 0;
      for (int i = 0; i < MIN_MSE_COUNT; i++) {
        tmp32no1 = ((int32_t)g_aecm.echoStoredLogEnergy[i] -
                    (int32_t)g_aecm.nearLogEnergy[i]);
        tmp32no2 = ABS_W32(tmp32no1);
        mseStored += tmp32no2;

        tmp32no1 = ((int32_t)g_aecm.echoAdaptLogEnergy[i] -
                    (int32_t)g_aecm.nearLogEnergy[i]);
        tmp32no2 = ABS_W32(tmp32no1);
        mseAdapt += tmp32no2;
      }
      if (((mseStored << MSE_RESOLUTION) < (MIN_MSE_DIFF * mseAdapt)) &
          ((g_aecm.mseStoredOld << MSE_RESOLUTION) <
           (MIN_MSE_DIFF * g_aecm.mseAdaptOld))) {
        // 保存チャネルの方が連続して適応チャネルより低い誤差なら、
        // 適応チャネルをリセットする。
        ResetAdaptiveChannelC();
      } else if (((MIN_MSE_DIFF * mseStored) > (mseAdapt << MSE_RESOLUTION)) &
                 (mseAdapt < g_aecm.mseThreshold) &
                 (g_aecm.mseAdaptOld < g_aecm.mseThreshold)) {
        // 適応チャネルの方が連続して保存チャネルより低い誤差なら、
        // 
        // 適応チャネルを保存版として採用する。
        StoreAdaptiveChannelC(X_mag, S_mag);

        // 閾値を更新
        if (g_aecm.mseThreshold == WORD32_MAX) {
          g_aecm.mseThreshold = (mseAdapt + g_aecm.mseAdaptOld);
        } else {
          int scaled_threshold = g_aecm.mseThreshold * 5 / 8;
          g_aecm.mseThreshold += ((mseAdapt - scaled_threshold) * 205) >> 8;
        }
      }

      // カウンタをリセット
      g_aecm.mseChannelCount = 0;

      // MSE を記録する。
      g_aecm.mseStoredOld = mseStored;
      g_aecm.mseAdaptOld = mseAdapt;
    }
  }
  // チャネル保存/復元の判定ここまで
}


// Wiener フィルタで用いる抑圧ゲインを計算する。
// 
//
// @param  aecm     [i/n]   Handle of the AECM instance.
// @param  supGain  [out]   (Return value) Suppression gain with which to scale
// 
//                          level (Q14).
int16_t CalcSuppressionGain() {
  int32_t tmp32no1;

  int16_t supGain = SUPGAIN_DEFAULT;
  int16_t tmp16no1;
  int16_t dE = 0;

  // Wiener フィルタで使用する抑圧ゲインを決定。遠端エネルギーと
  // 推定エコー誤差の組み合わせに基づき、遠端信号レベルに応じて調整する。
  // 遠端レベルが低ければ信号無しとみなし、抑圧ゲインを 0 に近づける。
  // 
  if (!g_aecm.currentVADValue) {
    supGain = 0;
  } else {
    // ダブルトークの可能性に備えて調整する。誤差変動が大きければ
    // ダブルトーク（または悪いチャネル）とみなし、
    tmp16no1 = (g_aecm.nearLogEnergy[0] - g_aecm.echoStoredLogEnergy[0] -
                ENERGY_DEV_OFFSET);
    dE = ABS_W16(tmp16no1);

    if (dE < ENERGY_DEV_TOL) {
      // ダブルトークでなければ推定品質に応じて抑圧量を増やす
      // カウンタも更新
      if (dE < SUPGAIN_EPC_DT) {
        tmp32no1 = g_aecm.supGainErrParamDiffAB * dE;
        tmp32no1 += (SUPGAIN_EPC_DT >> 1);
        tmp16no1 = (int16_t)DivW32W16(tmp32no1, SUPGAIN_EPC_DT);
        supGain = g_aecm.supGainErrParamA - tmp16no1;
      } else {
        tmp32no1 = g_aecm.supGainErrParamDiffBD * (ENERGY_DEV_TOL - dE);
        tmp32no1 += ((ENERGY_DEV_TOL - SUPGAIN_EPC_DT) >> 1);
        tmp16no1 = (int16_t)DivW32W16(
            tmp32no1, (ENERGY_DEV_TOL - SUPGAIN_EPC_DT));
        supGain = g_aecm.supGainErrParamD + tmp16no1;
      }
    } else {
      // ダブルトークの兆候があれば既定値を適用
      supGain = g_aecm.supGainErrParamD;
    }
  }

  if (supGain > g_aecm.supGainOld) {
    tmp16no1 = supGain;
  } else {
    tmp16no1 = g_aecm.supGainOld;
  }
  g_aecm.supGainOld = supGain;
  if (tmp16no1 < g_aecm.supGain) {
    g_aecm.supGain += (int16_t)((tmp16no1 - g_aecm.supGain) >> 4);
  } else {
    g_aecm.supGain += (int16_t)((tmp16no1 - g_aecm.supGain) >> 4);
  }

  // 抑圧ゲイン更新ここまで

  return g_aecm.supGain;
}

void BufferFarFrame(const int16_t* const x_frame) {
  const int xLen = FRAME_LEN;
  int writeLen = xLen, writePos = 0;

  // 書き込み位置をリングバッファ境界で折り返すか確認
  while (g_aecm.xBufWritePos + writeLen > FAR_BUF_LEN) {
    // 折り返す前に残り領域へ書き込む
    writeLen = FAR_BUF_LEN - g_aecm.xBufWritePos;
    memcpy(g_aecm.xFrameBuf + g_aecm.xBufWritePos, x_frame + writePos,
           sizeof(int16_t) * writeLen);
    g_aecm.xBufWritePos = 0;
    writePos = writeLen;
    writeLen = xLen - writeLen;
  }

  memcpy(g_aecm.xFrameBuf + g_aecm.xBufWritePos, x_frame + writePos,
         sizeof(int16_t) * writeLen);
  g_aecm.xBufWritePos += writeLen;
}

void FetchFarFrame(int16_t* const x_frame, const int knownDelay) {
  const int xLen = FRAME_LEN;
  int readLen = xLen;
  int readPos = 0;
  int delayChange = knownDelay - g_aecm.lastKnownDelay;

  g_aecm.xBufReadPos -= delayChange;

  // 遅延調整で読取位置が範囲外なら補正
  while (g_aecm.xBufReadPos < 0) {
    g_aecm.xBufReadPos += FAR_BUF_LEN;
  }
  while (g_aecm.xBufReadPos > FAR_BUF_LEN - 1) {
    g_aecm.xBufReadPos -= FAR_BUF_LEN;
  }

  g_aecm.lastKnownDelay = knownDelay;

  // 読取位置も境界で折り返すか確認
  while (g_aecm.xBufReadPos + readLen > FAR_BUF_LEN) {
    // 折り返す前に残り領域から読み出す
    readLen = FAR_BUF_LEN - g_aecm.xBufReadPos;
    memcpy(x_frame + readPos, g_aecm.xFrameBuf + g_aecm.xBufReadPos,
           sizeof(int16_t) * readLen);
    g_aecm.xBufReadPos = 0;
    readPos = readLen;
    readLen = xLen - readLen;
  }
  memcpy(x_frame + readPos, g_aecm.xFrameBuf + g_aecm.xBufReadPos,
         sizeof(int16_t) * readLen);
  g_aecm.xBufReadPos += readLen;
}

 
