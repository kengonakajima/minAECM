#include "aecm.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "delay_estimator.h"
#include "util.h"

#ifdef MSC_VER  // Visual C++
#define ALIGN8_BEG __declspec(align(8))
#define ALIGN8_END
#else  // gcc または icc
#define ALIGN8_BEG
#define ALIGN8_END __attribute__((aligned(8)))
#endif

bool g_firstVAD; // VAD 初回検出フラグ。検出済みならtrue
uint16_t g_xHistory[PART_LEN1 * MAX_DELAY]; // 遠端スペクトル履歴（遅延候補ごと）
int g_xHistoryPos; // 遠端スペクトル履歴の書き込みインデックス


uint32_t g_totCount; // 処理済みブロック数のカウンタ

// dfa: Dynamic Fixed-point Alignment
int16_t g_dfaCleanQDomain; // クリーン成分の Q-domain 推定値
int16_t g_dfaCleanQDomainOld; // 上記の1ブロック前の値
int16_t g_dfaNoisyQDomain; // 雑音成分の Q-domain 推定値
int16_t g_dfaNoisyQDomainOld; // 雑音 Q-domain の1ブロック前の値


int16_t g_nearLogEnergy[MAX_LOG_LEN]; // 近端信号の対数エネルギー履歴
int16_t g_farLogEnergy; // 遠端信号の対数エネルギー最新値
int16_t g_echoAdaptLogEnergy[MAX_LOG_LEN]; // 適応エコーパスによる対数エネルギー履歴
int16_t g_echoStoredLogEnergy[MAX_LOG_LEN]; // 保存エコーパスによる対数エネルギー履歴


int16_t g_HStored[PART_LEN1]; // 保存エコーパス係数（Q15）
int16_t g_HAdapt16[PART_LEN1]; // 適応エコーパス係数（Q15）
int32_t g_HAdapt32[PART_LEN1]; // 適応エコーパス係数（拡張Q31）
int16_t g_xBuf[PART_LEN2]; // 遠端時間領域バッファ（FFT入力）
int16_t g_yBuf[PART_LEN2]; // 近端時間領域バッファ（FFT入力）
int16_t g_eOverlapBuf[PART_LEN]; // IFFT のオーバーラップ保存領域

int32_t g_sMagSmooth[PART_LEN1]; // 推定エコー振幅の平滑値
int16_t g_yMagSmooth[PART_LEN1]; // 近端スペクトル振幅の平滑値


int32_t g_mseAdaptOld; // 適応チャネルの過去 MSE
int32_t g_mseStoredOld; // 保存チャネルの過去 MSE
int32_t g_mseThreshold; // MSE ベースのしきい値（可変）

int16_t g_farEnergyMin; // 遠端エネルギーの最小値トラッカ
int16_t g_farEnergyMax; // 遠端エネルギーの最大値トラッカ
int16_t g_farEnergyMaxMin; // 遠端エネルギーのレンジ指標
int16_t g_farEnergyVAD; // 遠端 VAD 用エネルギーしきい値
int16_t g_farEnergyMSEThres; // MSE 判定をするための遠端エネルギー基準値
bool g_currentVAD; // 近端 VAD の現在のフラグ。声があるならtrue
int16_t g_vadUpdateCount; // VAD 関連の更新カウンタ

int16_t g_startupState; // 起動フェーズの状態
int16_t g_mseChannelCount; // MSE 判定でのチャネル更新回数
int16_t g_supGain; // 現在の抑圧ゲイン（Q8）
int16_t g_supGainOld; // 直前の抑圧ゲイン（Q8）


// ハニング窓の平方根（Q14）。
static const ALIGN8_BEG int16_t kSqrtHanning[] ALIGN8_END = {
    0,     399,   798,   1196,  1594,  1990,  2386,  2780,  3172,  3562,  3951,
    4337,  4720,  5101,  5478,  5853,  6224,  6591,  6954,  7313,  7668,  8019,
    8364,  8705,  9040,  9370,  9695,  10013, 10326, 10633, 10933, 11227, 11514,
    11795, 12068, 12335, 12594, 12845, 13089, 13325, 13553, 13773, 13985, 14189,
    14384, 14571, 14749, 14918, 15079, 15231, 15373, 15506, 15631, 15746, 15851,
    15947, 16034, 16111, 16179, 16237, 16286, 16325, 16354, 16373, 16384};

// 近似版の振幅計算は使わず、sqrtベースのみを使用
static bool g_bypass_wiener = false;
static bool g_bypass_nlp = false;

void SetBypassWiener(int enable) {
  g_bypass_wiener = (enable != 0);
}

void SetBypassNlp(int enable) {
  g_bypass_nlp = (enable != 0);
}


// 非対称フィルタ処理を行う。
//
// 入力:
//      - filtOld       : Previous filtered value.
//      - inVal         : New input value.
//      - stepSizePos   : Step size when we have a positive contribution.
//      - stepSizeNeg   : Step size when we have a negative contribution.
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

void TimeToFrequencyDomain(const int16_t* time_signal,
                           ComplexInt16* freq_signal,
                           uint16_t* freq_signal_abs,
                           uint32_t* freq_signal_sum_abs) {
  int16_t fft[PART_LEN4];

  WindowAndFFT(fft, time_signal, freq_signal, PART_LEN, kSqrtHanning);

  // 実部と虚部を取り出し、各ビンの振幅を計算
  freq_signal[0].imag = 0;
  freq_signal[PART_LEN].imag = 0;
  freq_signal_abs[0] = (uint16_t)ABS_W16(freq_signal[0].real);
  freq_signal_abs[PART_LEN] = (uint16_t)ABS_W16(freq_signal[PART_LEN].real);
  (*freq_signal_sum_abs) = (uint32_t)(freq_signal_abs[0]) + (uint32_t)(freq_signal_abs[PART_LEN]);

  for (int i = 1; i < PART_LEN; i++) {
    if (freq_signal[i].real == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].imag);
    } else if (freq_signal[i].imag == 0) {
      freq_signal_abs[i] = (uint16_t)ABS_W16(freq_signal[i].real);
    } else {
      int16_t abs_real = ABS_W16(freq_signal[i].real);
      int16_t abs_imag = ABS_W16(freq_signal[i].imag);
      int32_t sq_real = abs_real * abs_real;
      int32_t sq_imag = abs_imag * abs_imag;
      int32_t sum_sq = AddSatW32(sq_real, sq_imag);
      int32_t mag = SqrtFloor(sum_sq);
      freq_signal_abs[i] = (uint16_t)mag;
    }
    (*freq_signal_sum_abs) += (uint32_t)freq_signal_abs[i];
  }
}


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
void InitEchoPath(const int16_t* echo_path) {
  // 保存チャネルをリセット
  memcpy(g_HStored, echo_path, sizeof(int16_t) * PART_LEN1);
  // 適応チャネルをリセット
  memcpy(g_HAdapt16, echo_path, sizeof(int16_t) * PART_LEN1);
  for (int i = 0; i < PART_LEN1; i++) {
    g_HAdapt32[i] = (int32_t)g_HAdapt16[i] << 16;
  }

  // チャネル保存に関する変数を初期化
  g_mseAdaptOld = 1000;
  g_mseStoredOld = 1000;
  g_mseThreshold = WORD32_MAX;
  g_mseChannelCount = 0;
}

// H_adapt(Q15) → H_stored にコピーして、新しい S_mag を再計算
void StoreAdaptiveChannel(const uint16_t* X_mag, int32_t* S_mag) {
  // 起動中は毎ブロック保存チャネルを更新
  memcpy(g_HStored, g_HAdapt16, sizeof(int16_t) * PART_LEN1);
  // 推定エコーを再計算
  for (int i = 0; i < PART_LEN; i += 4) {
    S_mag[i] = MUL_16_U16(g_HStored[i], X_mag[i]);
    S_mag[i + 1] = MUL_16_U16(g_HStored[i + 1], X_mag[i + 1]);
    S_mag[i + 2] = MUL_16_U16(g_HStored[i + 2], X_mag[i + 2]);
    S_mag[i + 3] = MUL_16_U16(g_HStored[i + 3], X_mag[i + 3]);
  }
  // PART_LEN1 は PART_LEN + 1
  S_mag[PART_LEN] = MUL_16_U16(g_HStored[PART_LEN], X_mag[PART_LEN]);
}

void ResetAdaptiveChannel() {
  // 連続 2 回、保存チャネルの MSE が適応チャネルより十分小さい場合、
  // 適応チャネルをリセットする。
  memcpy(g_HAdapt16, g_HStored, sizeof(int16_t) * PART_LEN1);
  // 32bit チャネル表現を復元
  for (int i = 0; i < PART_LEN; i += 4) {
    g_HAdapt32[i] = (int32_t)g_HStored[i] << 16;
    g_HAdapt32[i + 1] = (int32_t)g_HStored[i + 1] << 16;
    g_HAdapt32[i + 2] = (int32_t)g_HStored[i + 2] << 16;
    g_HAdapt32[i + 3] = (int32_t)g_HStored[i + 3] << 16;
  }
  g_HAdapt32[PART_LEN] = (int32_t)g_HStored[PART_LEN] << 16;
}


void InitAecm() {
  // 16kHz 固定
  memset(g_xBuf, 0, sizeof(g_xBuf));
  memset(g_yBuf, 0, sizeof(g_yBuf));
  memset(g_eOverlapBuf, 0, sizeof(g_eOverlapBuf));

  g_totCount = 0;

  InitDelayEstimatorFarend();
  InitDelayEstimator();
  // 遠端履歴をゼロ初期化
  memset(g_xHistory, 0, sizeof(uint16_t) * PART_LEN1 * MAX_DELAY);
  g_xHistoryPos = MAX_DELAY;

  g_dfaCleanQDomain = 0;
  g_dfaCleanQDomainOld = 0;
  g_dfaNoisyQDomain = 0;
  g_dfaNoisyQDomainOld = 0;

  memset(g_nearLogEnergy, 0, sizeof(g_nearLogEnergy));
  g_farLogEnergy = 0;
  memset(g_echoAdaptLogEnergy, 0, sizeof(g_echoAdaptLogEnergy));
  memset(g_echoStoredLogEnergy, 0, sizeof(g_echoStoredLogEnergy));

  // エコーチャネルを既定形状（16 kHz 固定）で初期化
  InitEchoPath(kChannelStored16kHz);

  memset(g_sMagSmooth, 0, sizeof(g_sMagSmooth));
  memset(g_yMagSmooth, 0, sizeof(g_yMagSmooth));

  g_farEnergyMin = WORD16_MAX;
  g_farEnergyMax = WORD16_MIN;
  g_farEnergyMaxMin = 0;
  g_farEnergyVAD = FAR_ENERGY_MIN;  // 開始直後の誤検出（音声とみなさない）を防ぐ
                                        // 
  g_farEnergyMSEThres = 0;
  g_currentVAD = false;
  g_vadUpdateCount = 0;
  g_firstVAD = true;

  g_startupState = 0;
  g_supGain = SUPGAIN_DEFAULT;
  g_supGainOld = SUPGAIN_DEFAULT;

  // コンパイル時に前提条件を static_assert で確認
  // アセンブリ実装が依存するため、修正時は該当ファイルを要確認。
  static_assert(PART_LEN % 16 == 0, "PART_LEN is not a multiple of 16");

  static_assert(kRealFftOrder == PART_LEN_SHIFT, "FFT order と PART_LEN_SHIFT が不一致です");

}



// `a` の仮数部を、先頭ゼロ数 `zeros` を加味した Q8 の int16_t として返す。
// ゼロ数との整合性チェックは行わない。
// 
// NLMS によるチャネル推定と保存判定。
// X_mag: 遠端振幅スペクトル(Q0)、Y_mag: 近端振幅スペクトル(Q0)、
// mu: 上記で算出したシフト量、S_mag: 推定エコー（Q=RESOLUTION_CHANNEL16）。
void UpdateChannel(const uint16_t* X_mag,
                              const uint16_t* const Y_mag,
                              const int16_t mu,
                              int32_t* S_mag) {
  uint32_t channel_far_product_q0, near_mag_q0;
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
      zerosCh = NormU32(g_HAdapt32[i]);
      zerosFar = NormU32((uint32_t)X_mag[i]);
      if (zerosCh + zerosFar > 31) {
        // 乗算しても安全な状態
        channel_far_product_q0 = UMUL_32_16(g_HAdapt32[i], X_mag[i]);
        shiftChFar = 0;
      } else {
        // 乗算前にシフトダウンが必要
        shiftChFar = 32 - zerosCh - zerosFar;
        // zerosCh==zerosFar==0 なら shiftChFar=32 となり、
        // 右シフト 32 は未定義なのでチェックする。
        {
          uint32_t shifted = (shiftChFar >= 32)
                                  ? 0u
                                  : (uint32_t)(g_HAdapt32[i] >> shiftChFar);
          channel_far_product_q0 = shifted * X_mag[i];
        }
      }
      // 分子の Q ドメインを決定
      zerosNum = NormU32(channel_far_product_q0);
      if (Y_mag[i]) {
        zerosDfa = NormU32((uint32_t)Y_mag[i]);
      } else {
        zerosDfa = 32;
      }
      tmp16no1 = zerosDfa - 2 + g_dfaNoisyQDomain - RESOLUTION_CHANNEL32 + shiftChFar;
      if (zerosNum > tmp16no1 + 1) {
        xfaQ = tmp16no1;
        yMagQ = zerosDfa - 2;
      } else {
        xfaQ = zerosNum - 2;
        yMagQ = RESOLUTION_CHANNEL32 - g_dfaNoisyQDomain - shiftChFar + xfaQ;
      }
      // 同じ Q ドメインに揃えて加算
      channel_far_product_q0 = SHIFT_W32(channel_far_product_q0, xfaQ);
      near_mag_q0 = SHIFT_W32((uint32_t)Y_mag[i], yMagQ);
      int32_t residual_q31 = (int32_t)near_mag_q0 - (int32_t)channel_far_product_q0;
      zerosNum = NormW32(residual_q31);
      if (residual_q31 && (X_mag[i] > CHANNEL_VAD)) {
        //
        // 更新が必要なケース
        //
        // 以下の計算を行いたい：
        //
        // residual = Y_mag[i] - (H_adapt[i] * X_mag[i])
        // tmp32norm = (i + 1)
        // aecm->channelAdapt[i] += (2^mu) * residual
        //                        / (tmp32norm * X_mag[i])
        //

        // 乗算でオーバーフローしないようにする。
        int32_t residual_times_far;
        if (zerosNum + zerosFar > 31) {
          if (residual_q31 > 0) {
            residual_times_far = (int32_t)UMUL_32_16(residual_q31, X_mag[i]);
          } else {
            residual_times_far = -(int32_t)UMUL_32_16(-residual_q31, X_mag[i]);
          }
          shiftNum = 0;
        } else {
          shiftNum = 32 - (zerosNum + zerosFar);
          if (residual_q31 > 0) {
            residual_times_far = (residual_q31 >> shiftNum) * X_mag[i];
          } else {
            residual_times_far = -((-residual_q31 >> shiftNum) * X_mag[i]);
          }
        }
        // 周波数ビンに応じて正規化
        residual_times_far = DivW32W16(residual_times_far, i + 1);
        // 適切な Q ドメインに揃える
        shift2ResChan = shiftNum + shiftChFar - xfaQ - mu - ((30 - zerosFar) << 1);
        int32_t gradient_q31;
        if (NormW32(residual_times_far) < shift2ResChan) {
          gradient_q31 = WORD32_MAX;
        } else {
          gradient_q31 = SHIFT_W32(residual_times_far, shift2ResChan);
        }
        g_HAdapt32[i] = AddSatW32(g_HAdapt32[i], gradient_q31);
        if (g_HAdapt32[i] < 0) {
          // チャネル利得が負にならないよう強制
          g_HAdapt32[i] = 0;
        }
        g_HAdapt16[i] = (int16_t)(g_HAdapt32[i] >> 16);
      }
    }
  }
  // 適応チャネル更新ここまで


  // チャネルを保存するか復元するかを判定
  if ((g_startupState == 0) && g_currentVAD) {
    // 起動中は毎ブロックチャネルを保存し、
    // 推定エコーも再計算する
    StoreAdaptiveChannel(X_mag, S_mag);
  } else {
    if (g_farLogEnergy < g_farEnergyMSEThres) {
      g_mseChannelCount = 0;
    } else {
      g_mseChannelCount++;
    }
    // 検証に十分なデータがあれば、保存を検討
    if (g_mseChannelCount >= (MIN_MSE_COUNT + 10)) {
      // 十分なデータが揃った
      // 適応版と保存版の MSE を計算
      // 実際には平均絶対誤差に近い指標
      mseStored = 0;
      mseAdapt = 0;
      for (int i = 0; i < MIN_MSE_COUNT; i++) {
        int32_t stored_error_q8 =
            static_cast<int32_t>(g_echoStoredLogEnergy[i]) -
            static_cast<int32_t>(g_nearLogEnergy[i]);
        int32_t stored_error_abs_q8 = ABS_W32(stored_error_q8);
        mseStored += stored_error_abs_q8;

        int32_t adapt_error_q8 =
            static_cast<int32_t>(g_echoAdaptLogEnergy[i]) -
            static_cast<int32_t>(g_nearLogEnergy[i]);
        int32_t adapt_error_abs_q8 = ABS_W32(adapt_error_q8);
        mseAdapt += adapt_error_abs_q8;
      }
      if (((mseStored << MSE_RESOLUTION) < (MIN_MSE_DIFF * mseAdapt)) &
          ((g_mseStoredOld << MSE_RESOLUTION) <
           (MIN_MSE_DIFF * g_mseAdaptOld))) {
        // 保存チャネルの方が連続して適応チャネルより低い誤差なら、
        // 適応チャネルをリセットする。
        ResetAdaptiveChannel();
      } else if (((MIN_MSE_DIFF * mseStored) > (mseAdapt << MSE_RESOLUTION)) &
                 (mseAdapt < g_mseThreshold) &
                 (g_mseAdaptOld < g_mseThreshold)) {
        // 適応チャネルの方が連続して保存チャネルより低い誤差なら、
        // 
        // 適応チャネルを保存版として採用する。
        StoreAdaptiveChannel(X_mag, S_mag);

        // 閾値を更新
        if (g_mseThreshold == WORD32_MAX) {
          g_mseThreshold = (mseAdapt + g_mseAdaptOld);
        } else {
          int scaled_threshold = g_mseThreshold * 5 / 8;
          g_mseThreshold += ((mseAdapt - scaled_threshold) * 205) >> 8;
        }
      }

      // カウンタをリセット
      g_mseChannelCount = 0;

      // MSE を記録する。
      g_mseStoredOld = mseStored;
      g_mseAdaptOld = mseAdapt;
    }
  }
}

// blockは64サンプルの時間領域データ。符号付き線形PCM -32768 ~ 32767
// x_block: 遠端, y_block: 近端, e_block: キャンセル済みの残差信号
int ProcessBlock(const int16_t* x_block, const int16_t* y_block, int16_t* e_block) {
  // スタートアップ状態を判定する。段階は次の 3 つ:
  // (0) 最初の CONV_LEN ブロック
  // (1) さらに CONV_LEN ブロック
  // (2) それ以降

  if (g_startupState < 2) {
    g_startupState = (g_totCount >= CONV_LEN) + (g_totCount >= CONV_LEN2);
  }

  // 1. ブロック入力とバッファ更新 x: x_block y: y_block
  // 近端/遠端の時間領域フレームをバッファへ蓄える
  memcpy(g_xBuf + PART_LEN, x_block, sizeof(int16_t) * PART_LEN);
  memcpy(g_yBuf + PART_LEN, y_block, sizeof(int16_t) * PART_LEN);

  // 2. 時間領域から周波数領域に変換. Y_freqは捨てる。
  ComplexInt16 Y_freq[PART_LEN2]; // Y の周波数領域表現
  uint16_t X_mag[PART_LEN1]; // |X| Xの絶対値スペクトル
  uint16_t Y_mag[PART_LEN1]; // |Y| Yの絶対値スペクトル
  uint32_t X_mag_sum = 0; // sum(|X|) 遠端のエネルギー
  TimeToFrequencyDomain(g_xBuf, Y_freq, X_mag, &X_mag_sum); // |X|, sum(|X|) = FFT(x)
  uint32_t Y_mag_sum = 0;
  TimeToFrequencyDomain(g_yBuf, Y_freq, Y_mag, &Y_mag_sum); // Y, |Y|, sum(|Y|) = FFT(y)

  g_xHistoryPos++;
  if (g_xHistoryPos >= MAX_DELAY) {
    g_xHistoryPos = 0;
  }
  memcpy(&(g_xHistory[g_xHistoryPos * PART_LEN1]), X_mag, sizeof(uint16_t) * PART_LEN1); // |X|を履歴に積む

  // 3. 2値スペクトル履歴からブロック単位の遅延を推定する。
  int delay = DelayEstimatorProcess(Y_mag,X_mag); // delay : 整数値。単位はブロック
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    delay = 0;  // 遅延が不明な場合は 0 と仮定する。
  }

  // 推定した遅延に合わせて遠端スペクトルを整列する。整列とは処理対象とするブロックを選ぶこと。
  int buffer_position = g_xHistoryPos - delay;
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  const uint16_t* X_mag_aligned = &(g_xHistory[buffer_position * PART_LEN1]); // |X_aligned|

  // 4. 対数表現エネルギー4種類の履歴を更新
  uint32_t tmpFar = 0;  // 遠端スペクトル|X(k)|の総和、計算用
  uint32_t tmpAdapt = 0;  // 適応チャネル出力|S_hat_adapt(k)|の総和、計算用
  uint32_t tmpStored = 0;  // 保存チャネル出力|S_hat_stored(k)|の総和、計算用
  int16_t tmp16;
  int16_t increase_max_shifts = 4;
  int16_t decrease_max_shifts = 11;
  int16_t increase_min_shifts = 11;
  int16_t decrease_min_shifts = 3;

  g_dfaNoisyQDomainOld = g_dfaNoisyQDomain;
  g_dfaNoisyQDomain = 0;

  g_dfaCleanQDomainOld = g_dfaNoisyQDomainOld;
  g_dfaCleanQDomain = g_dfaNoisyQDomain;
  
  // 近端の対数エネルギー履歴を後ろに1つずらす
  memmove(g_nearLogEnergy + 1, g_nearLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));
  g_nearLogEnergy[0] = LogOfEnergyInQ8(Y_mag_sum, g_dfaNoisyQDomain);

  int32_t S_mag[PART_LEN1]; // |Ŝ(k)|: 予測エコー振幅（チャネル通過後）    
  for (int i = 0; i < PART_LEN1; i++) {
      S_mag[i] = MUL_16_U16(g_HStored[i], X_mag_aligned[i]); // 推定エコー信号Sを計算
      tmpFar += (uint32_t)X_mag_aligned[i]; // 遠端エネルギー総和
      tmpAdapt += g_HAdapt16[i] * X_mag_aligned[i];  // 学習中チャネルによるエコーエネルギー
      tmpStored += (uint32_t)S_mag[i]; // S_magのエネルギー
  }

  // 対数エネルギー履歴バッファを後ろに1つずらす
  memmove(g_echoAdaptLogEnergy + 1, g_echoAdaptLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));
  memmove(g_echoStoredLogEnergy + 1, g_echoStoredLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));

  g_farLogEnergy = LogOfEnergyInQ8(tmpFar, 0);
  g_echoAdaptLogEnergy[0] = LogOfEnergyInQ8(tmpAdapt, RESOLUTION_CHANNEL16); // 後ろにずらしたので[0]に代入可能。
  g_echoStoredLogEnergy[0] = LogOfEnergyInQ8(tmpStored, RESOLUTION_CHANNEL16); // 後ろにずらしたので[0]に代入可能。

  // 5. 遠端のエネルギーを評価する
  if (g_farLogEnergy > FAR_ENERGY_MIN) {
      if (g_startupState == 0) {
          increase_max_shifts = 2;
          decrease_min_shifts = 2;
          increase_min_shifts = 8;
      }
      // AsymFilt: 上昇は速く、下降は遅くする非対称フィルタ
      g_farEnergyMin = AsymFilt(g_farEnergyMin, g_farLogEnergy, increase_min_shifts, decrease_min_shifts);
      g_farEnergyMax = AsymFilt(g_farEnergyMax, g_farLogEnergy, increase_max_shifts, decrease_max_shifts);
      g_farEnergyMaxMin = g_farEnergyMax - g_farEnergyMin;

      // VAD判定用の閾値 g_farEnergyVAD を更新する。
      tmp16 = 2560 - g_farEnergyMin;
      if (tmp16 > 0) {
          tmp16 = static_cast<int16_t>((tmp16 * FAR_ENERGY_VAD_REGION) >> 9);
      } else {
          tmp16 = 0;
      }
      tmp16 += FAR_ENERGY_VAD_REGION;

      if ((g_startupState == 0) | (g_vadUpdateCount > 1024)) { // 起動直後と、長期間一定だったときはリセットする
          g_farEnergyVAD = g_farEnergyMin + tmp16;
      } else {
          if (g_farEnergyVAD > g_farLogEnergy) {
              g_farEnergyVAD += (g_farLogEnergy + tmp16 - g_farEnergyVAD) >> 6; // 遠端が閾値より小さいときはゆっくり更新する。カウンタはゆっくりにする用
              g_vadUpdateCount = 0;
          } else {
              g_vadUpdateCount++;
          }
      }
      g_farEnergyMSEThres = g_farEnergyVAD + (1 << 8); // この値を後段8でHの昇格判定に使う。
  }

  // VADの結論を出す
  if (g_farLogEnergy > g_farEnergyVAD) { 
      if ((g_startupState == 0) | (g_farEnergyMaxMin > FAR_ENERGY_DIFF)) {
          g_currentVAD = true; // 声がある
      }
  } else {
      g_currentVAD = false; // 声がない
  }
  if (g_currentVAD && g_firstVAD) { // 最初に声が入ったか?
      g_firstVAD = false;
      if (g_echoAdaptLogEnergy[0] > g_nearLogEnergy[0]) {
          for (int i = 0; i < PART_LEN1; i++) {
              g_HAdapt16[i] >>= 3;
          }
          g_echoAdaptLogEnergy[0] -= (3 << 8);
          g_firstVAD = true;
      }
  }

  // 6. NLMSステップサイズ μ の計算
  const int MU_MIN = 10;  // 遠端エネルギーに依存する最小ステップ（2^-MU_MIN） 
  const int MU_MAX = 1;   // 遠端エネルギーに依存する最大ステップ（2^-MU_MAX） 
  const int MU_DIFF = 9;  // MU_MIN と MU_MAX の差   
  int16_t mu = MU_MAX;
  if (!g_currentVAD) { 
    mu = 0; // 声がないときは、全く学習しない。
  } else if (g_startupState > 0) {
    if (g_farEnergyMin >= g_farEnergyMax) { // 初期化直後はこの値がブレることがある。そのときに混乱しないため
      mu = MU_MIN; 
    } else {
      int16_t mu_tmp16 = g_farLogEnergy - g_farEnergyMin;
      int32_t mu_tmp32 = mu_tmp16 * MU_DIFF;
      mu_tmp32 = DivW32W16(mu_tmp32, g_farEnergyMaxMin);
      mu = static_cast<int16_t>(MU_MIN - 1 - mu_tmp32);
    }
    if (mu < MU_MAX) {
      mu = MU_MAX;
    }
  }

  // 処理済みブロック数をインクリメント
  g_totCount++;

  // 7. エコーチャネル更新
  // NLMSに似た方法で、 muを用いて Hadaptを更新する
  UpdateChannel(X_mag_aligned, Y_mag, mu, S_mag);

  int16_t supGain = SUPGAIN_DEFAULT;
  int16_t tmp16no1;
  int16_t dE = 0;

  if (!g_currentVAD) {
      supGain = 0;
  } else {
      tmp16no1 = (g_nearLogEnergy[0] - g_echoStoredLogEnergy[0] - ENERGY_DEV_OFFSET);
      dE = ABS_W16(tmp16no1);

      if (dE < ENERGY_DEV_TOL) {
          if (dE < SUPGAIN_EPC_DT) {
              const int diffAB = SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
              int32_t sup_gain_numerator = diffAB * dE;
              sup_gain_numerator += (SUPGAIN_EPC_DT >> 1);
              tmp16no1 = (int16_t)DivW32W16(sup_gain_numerator, SUPGAIN_EPC_DT);
              supGain = SUPGAIN_ERROR_PARAM_A - tmp16no1;
          } else {
              const int diffBD = SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;
              int32_t sup_gain_numerator = diffBD * (ENERGY_DEV_TOL - dE);
              sup_gain_numerator += ((ENERGY_DEV_TOL - SUPGAIN_EPC_DT) >> 1);
              tmp16no1 = (int16_t)DivW32W16(sup_gain_numerator, (ENERGY_DEV_TOL - SUPGAIN_EPC_DT));
              supGain = SUPGAIN_ERROR_PARAM_D + tmp16no1;
          }
      } else {
          supGain = SUPGAIN_ERROR_PARAM_D;
      }
  }

  if (supGain > g_supGainOld) {
      tmp16no1 = supGain;
  } else {
      tmp16no1 = g_supGainOld;
  }
  g_supGainOld = supGain;
  if (tmp16no1 < g_supGain) {
      g_supGain += (int16_t)((tmp16no1 - g_supGain) >> 4);
  } else {
      g_supGain += (int16_t)((tmp16no1 - g_supGain) >> 4);
  }
  // 抑圧ゲイン更新ここまで


  // Wiener/NLP 用の抑圧マスク G(k) を算出
  int16_t G_mask[PART_LEN1];
  int16_t numPosCoef = 0;
  double sum_gain = 0.0;
  for (int i = 0; i < PART_LEN1; i++) {
    int32_t smooth_error_q31 = S_mag[i] - g_sMagSmooth[i];
    g_sMagSmooth[i] += (int32_t)(((int64_t)smooth_error_q31 * 50) >> 8);

    int16_t zeros32 = NormW32(g_sMagSmooth[i]) + 1;
    int16_t zeros16 = NormW16(g_supGain) + 1;
    uint32_t S_magGained;
    int16_t resolutionDiff;
    if (zeros32 + zeros16 > 16) {
      S_magGained = UMUL_32_16((uint32_t)g_sMagSmooth[i], (uint16_t)g_supGain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
    } else {
      int16_t tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff = 14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      if (zeros32 > tmp16no1) {
        S_magGained = UMUL_32_16((uint32_t)g_sMagSmooth[i], g_supGain >> tmp16no1);
      } else {
        S_magGained = (g_sMagSmooth[i] >> tmp16no1) * g_supGain;
      }
    }

    zeros16 = NormW16(g_yMagSmooth[i]);
    int16_t y_mag_q_domain_diff = g_dfaCleanQDomain - g_dfaCleanQDomainOld;
    int16_t qDomainDiff;
    int16_t tmp16no1;
    int16_t tmp16no2;
    if (zeros16 < y_mag_q_domain_diff && g_yMagSmooth[i]) {
      tmp16no1 = g_yMagSmooth[i] * (1 << zeros16);
      qDomainDiff = zeros16 - y_mag_q_domain_diff;
      tmp16no2 = Y_mag[i] >> -qDomainDiff;
    } else {
      tmp16no1 = y_mag_q_domain_diff < 0
                     ? g_yMagSmooth[i] >> -y_mag_q_domain_diff
                     : g_yMagSmooth[i] * (1 << y_mag_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = Y_mag[i];
    }
    int32_t near_mag_delta_q15 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(near_mag_delta_q15 >> 4);
    tmp16no2 += tmp16no1;
    zeros16 = NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      g_yMagSmooth[i] = WORD16_MAX;
    } else {
      g_yMagSmooth[i] = qDomainDiff < 0 ? tmp16no2 * (1 << -qDomainDiff)
                                             : tmp16no2 >> qDomainDiff;
    }

    if (S_magGained == 0) {
      G_mask[i] = ONE_Q14;
    } else if (g_yMagSmooth[i] == 0) {
      G_mask[i] = 0;
    } else {
      S_magGained += (uint32_t)(g_yMagSmooth[i] >> 1);
      uint32_t tmpU32 = DivU32U16(S_magGained, (uint16_t)g_yMagSmooth[i]);

      int32_t ratio_q14 = (int32_t)SHIFT_W32(tmpU32, resolutionDiff);
      if (ratio_q14 > ONE_Q14) {
        G_mask[i] = 0;
      } else if (ratio_q14 < 0) {
        G_mask[i] = ONE_Q14;
      } else {
        G_mask[i] = ONE_Q14 - (int16_t)ratio_q14;
        if (G_mask[i] < 0) {
          G_mask[i] = 0;
        }
      }
    }
    if (G_mask[i]) {
      numPosCoef++;
    }
  }
  if (g_bypass_wiener) {
    for (int i = 0; i < PART_LEN1; ++i) {
      G_mask[i] = ONE_Q14;
    }
    numPosCoef = PART_LEN1;
  }
  for (int i = 0; i < PART_LEN1; i++) {
    G_mask[i] = (int16_t)((G_mask[i] * G_mask[i]) >> 14);
  }

  const int kMinPrefBand = 4;
  const int kMaxPrefBand = 24;
  int32_t avgG32 = 0;
  for (int i = kMinPrefBand; i <= kMaxPrefBand; i++) {
    avgG32 += (int32_t)G_mask[i];
  }
  avgG32 /= (kMaxPrefBand - kMinPrefBand + 1);

  for (int i = kMaxPrefBand; i < PART_LEN1; i++) {
    if (G_mask[i] > (int16_t)avgG32) {
      G_mask[i] = (int16_t)avgG32;
    }
  }

  ComplexInt16 E_freq[PART_LEN2];
  for (int i = 0; i < PART_LEN1; i++) {
    if (G_mask[i] > NLP_COMP_HIGH) {
      G_mask[i] = ONE_Q14;
    } else if (G_mask[i] < NLP_COMP_LOW) {
      G_mask[i] = 0;
    }

    int16_t nlpGain = (numPosCoef < 3) ? 0 : ONE_Q14;
    if (g_bypass_nlp) {
      nlpGain = ONE_Q14;
    }

    if ((G_mask[i] == ONE_Q14) && (nlpGain == ONE_Q14)) {
      G_mask[i] = ONE_Q14;
    } else {
      G_mask[i] = (int16_t)((G_mask[i] * nlpGain) >> 14);
    }

    E_freq[i].real = (int16_t)(MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].real, G_mask[i], 14));
    E_freq[i].imag = (int16_t)(MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].imag, G_mask[i], 14));
    double gain_normalized = static_cast<double>(G_mask[i]) / static_cast<double>(ONE_Q14);
    sum_gain += gain_normalized;
  }

  // デバッグ出力
  double avg_gain = sum_gain / PART_LEN1;
  double suppression_db;
  if (avg_gain <= 0.0) avg_gain = 0.0;
  double clamped_gain = avg_gain;
  const double kMinGain = 1e-3;
  if (clamped_gain < kMinGain) clamped_gain = kMinGain;
  suppression_db = 20.0 * log10(clamped_gain);

  static int dbg_sup_counter = 0;
  static double best_gain = 1.0;
  static double best_db = 0.0;
  static int initialized = 0;
  dbg_sup_counter++;
  if (!initialized || suppression_db < best_db) {
      best_db = suppression_db;
      best_gain = clamped_gain;
      initialized = 1;
  }
  if (dbg_sup_counter % 100 == 0) {
      fprintf(stderr,
              "[Suppression] window=%d avg_gain=%.3f (%.1f dB)%s%s\n",
              dbg_sup_counter,
              best_gain,
              best_db,
              g_bypass_wiener ? " wiener-off" : "",
              g_bypass_nlp ? " nlp-off" : "");
      initialized = 0;
  }


  int16_t fft[PART_LEN4 + 2];
  int16_t time_current[PART_LEN];
  int16_t time_overlap[PART_LEN];
  InverseFFTAndWindow(fft,
                      E_freq,
                      PART_LEN,
                      PART_LEN2,
                      kSqrtHanning,
                      time_current,
                      time_overlap);

  for (int i = 0; i < PART_LEN; ++i) {
    int32_t tmp32 = (int32_t)time_current[i] + g_eOverlapBuf[i];
    e_block[i] = (int16_t)SAT(WORD16_MAX, tmp32, WORD16_MIN);
    g_eOverlapBuf[i] = time_overlap[i];
  }

  // 次ブロックで使用するため、最新フレームの後半を先頭へシフト
  memcpy(g_xBuf, g_xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(g_yBuf, g_yBuf + PART_LEN, sizeof(int16_t) * PART_LEN);


  // デバッグ出力
  static int dbg_ss_counter = 0;
  dbg_ss_counter++;
  if (dbg_ss_counter % 100 == 0) {
      fprintf(stderr, "[AECM] block=%d startupState=%d est_delay=%d\n",
              dbg_ss_counter, (int)g_startupState, delay);
  }

  return 0;
}
