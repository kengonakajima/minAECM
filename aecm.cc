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

struct ComplexInt16 {
  int16_t real;
  int16_t imag;
};

struct AecmCore {
  int xBufWritePos;
  int xBufReadPos;
  int knownDelay;
  int lastKnownDelay;
  int firstVAD;

  int16_t xFrameBuf[FAR_BUF_LEN];

  DelayEstimatorFarend delay_estimator_farend;
  DelayEstimator delay_estimator;
  uint16_t xHistory[PART_LEN1 * MAX_DELAY];
  int xHistoryPos;

  uint32_t totCount;

  int16_t dfaCleanQDomain;
  int16_t dfaCleanQDomainOld;
  int16_t dfaNoisyQDomain;
  int16_t dfaNoisyQDomainOld;

  int16_t nearLogEnergy[MAX_LOG_LEN];
  int16_t farLogEnergy;
  int16_t echoAdaptLogEnergy[MAX_LOG_LEN];
  int16_t echoStoredLogEnergy[MAX_LOG_LEN];

  int16_t hStored[PART_LEN1];
  int16_t hAdapt16[PART_LEN1];
  int32_t hAdapt32[PART_LEN1];
  int16_t xBuf[PART_LEN2];
  int16_t yBuf[PART_LEN2];
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
};

// デフォルトの単一インスタンス（ポインタ省略呼び出し用）
AecmCore g_aecm;

// アプリ側ラッパ状態（単一インスタンス）

constexpr int kBufSizeFrames = 50;  // 遠端バッファ長（フレーム数）
constexpr size_t kBufSizeSamples = kBufSizeFrames * FRAME_LEN;
constexpr int kSamplesPerMs16k = 16;  // 16 kHz 固定
constexpr short kFixedMsInSndCardBuf = 60;  // 50ms 遅延 + 10ms マージン
constexpr short kStartupFramesRaw =
    static_cast<short>((kFixedMsInSndCardBuf * kSamplesPerMs16k) / FRAME_LEN);
constexpr short kStartupFrames =
    kStartupFramesRaw <= 0
        ? 1
        : (kStartupFramesRaw > kBufSizeFrames ? kBufSizeFrames : kStartupFramesRaw);
constexpr int kInitCheck = 42;

struct AecMobile {
  short bufSizeStart;
  int knownDelay;
  short farendOld[FRAME_LEN];
  short initFlag;
  short filtDelay;
  int timeForDelayChange;
  int ECstartup;
  short lastDelayDiff;
  RingBuffer farendBuf;
  int16_t farendBufData[kBufSizeSamples];
};

static AecMobile g_mobile;


// 先行宣言（翻訳単位内のみで使用）。
void UpdateFarHistory(uint16_t* x_spectrum);
const uint16_t* AlignedFarX(int delay);
void CalcEnergies(const uint16_t* X_mag, uint32_t Y_energy, int32_t* S_mag);
int16_t CalcStepSize();
void UpdateChannel(const uint16_t* X_mag,
                   int16_t x_q,
                   const uint16_t* const Y_mag,
                   int16_t mu,
                   int32_t* S_mag);
int16_t CalcSuppressionGain();
void BufferFarFrame(const int16_t* const x_frame);
void FetchFarFrame(int16_t* const x_frame, int knownDelay);

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

static void SetBypassWienerInternal(int enable) {
  g_bypass_wiener = (enable != 0);
}

static void SetBypassNlpInternal(int enable) {
  g_bypass_nlp = (enable != 0);
}

void WindowAndFFT(int16_t* fft,
                  const int16_t* time_signal,
                  ComplexInt16* freq_signal) {
  // 信号に対して FFT を実行
  for (int i = 0; i < PART_LEN; i++) {
    // 時間領域信号に窓を掛け、変換配列 `fft` の実部へ格納
    int16_t scaled_time_signal = time_signal[i];
    fft[i] = (int16_t)((scaled_time_signal * kSqrtHanning[i]) >> 14);
    scaled_time_signal = time_signal[i + PART_LEN];
    fft[PART_LEN + i] = (int16_t)((scaled_time_signal * kSqrtHanning[PART_LEN - i]) >> 14);
  }

  // FFT を計算し、最初の PART_LEN 個の複素サンプルだけを保持
  // さらに虚部の符号を反転させる。
  RealForwardFFT(fft, (int16_t*)freq_signal);
  for (int i = 0; i < PART_LEN; i++) {
    freq_signal[i].imag = -freq_signal[i].imag;
  }
}

void InverseFFTAndWindow(int16_t* fft,
                         ComplexInt16* efw,
                         int16_t* output) {
  // `efw` の内容を `fft` に移した後の逆 FFT 出力バッファとして再利用
  int16_t* ifft_out = (int16_t*)efw;

  // 合成処理
  for (int i = 1, j = 2; i < PART_LEN; i += 1, j += 2) {
    fft[j] = efw[i].real;
    fft[j + 1] = -efw[i].imag;
  }
  fft[0] = efw[0].real;
  fft[1] = -efw[0].imag;

  fft[PART_LEN2] = efw[PART_LEN].real;
  fft[PART_LEN2 + 1] = -efw[PART_LEN].imag;

  // 逆 FFT を実行し、次ブロックでのスケール用に outCFFT を保持。
  int outCFFT = RealInverseFFT(fft, ifft_out);
  for (int i = 0; i < PART_LEN; i++) {
    ifft_out[i] = (int16_t)MUL_16_16_RSFT_WITH_ROUND(ifft_out[i], kSqrtHanning[i], 14);
    // 固定Q=0のため、出力シフトは outCFFT のみを考慮
    int32_t tmp32no1 = SHIFT_W32((int32_t)ifft_out[i], outCFFT);
    output[i] = (int16_t)SAT(WORD16_MAX, tmp32no1 + g_aecm.eOverlapBuf[i], WORD16_MIN);

    tmp32no1 = (ifft_out[PART_LEN + i] * kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = SHIFT_W32(tmp32no1, outCFFT);
    g_aecm.eOverlapBuf[i] = (int16_t)SAT(WORD16_MAX, tmp32no1, WORD16_MIN);
  }

  // 現ブロックの値を過去位置へコピーし、
  // （`g_aecm.eOverlapBuf` のシフトは別処理で行う）
  memcpy(g_aecm.xBuf, g_aecm.xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.yBuf, g_aecm.yBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
}

void TimeToFrequencyDomain(const int16_t* time_signal,
                           ComplexInt16* freq_signal,
                           uint16_t* freq_signal_abs,
                           uint32_t* freq_signal_sum_abs) {
  int16_t fft[PART_LEN4];

  WindowAndFFT(fft, time_signal, freq_signal);

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

int ProcessBlock(const int16_t* x_block, const int16_t* y_block, int16_t* e_block) {
  // 周波数領域バッファ（Y(k) の複素成分）
  ComplexInt16 Y_freq[PART_LEN2];
  // |X(k)|, |Y(k)| の絶対値スペクトル
  uint16_t X_mag[PART_LEN1];
  uint16_t Y_mag[PART_LEN1];
  // |Ŝ(k)|: 予測エコー振幅（チャネル通過後）
  int32_t S_mag[PART_LEN1];

  // スタートアップ状態を判定する。段階は次の 3 つ:
  // (0) 最初の CONV_LEN ブロック
  // (1) さらに CONV_LEN ブロック
  // (2) それ以降

  if (g_aecm.startupState < 2) {
    g_aecm.startupState = (g_aecm.totCount >= CONV_LEN) + (g_aecm.totCount >= CONV_LEN2);
  }

  // 近端/遠端の時間領域フレームをバッファへ蓄える
  memcpy(g_aecm.xBuf + PART_LEN, x_block, sizeof(int16_t) * PART_LEN);
  memcpy(g_aecm.yBuf + PART_LEN, y_block, sizeof(int16_t) * PART_LEN);

  // 遠端信号 X(k) = FFT{x(n)} を算出し（|X| と Σ|X| を求める）
  uint32_t X_mag_sum = 0;
  TimeToFrequencyDomain(g_aecm.xBuf, Y_freq, X_mag, &X_mag_sum);

  // 近端信号 Y(k) = FFT{y(n)} を算出し（|Y| と Σ|Y| を求める）
  uint32_t Y_mag_sum = 0;
  TimeToFrequencyDomain(g_aecm.yBuf, Y_freq, Y_mag, &Y_mag_sum);
  g_aecm.dfaNoisyQDomainOld = g_aecm.dfaNoisyQDomain;
  g_aecm.dfaNoisyQDomain = 0;

  g_aecm.dfaCleanQDomainOld = g_aecm.dfaNoisyQDomainOld;
  g_aecm.dfaCleanQDomain = g_aecm.dfaNoisyQDomain;

  // 遠端スペクトル履歴を更新し、2値スペクトル照合で遅延を推定
  UpdateFarHistory(X_mag);
  if (AddFarSpectrum(&g_aecm.delay_estimator_farend, X_mag) == -1) {
    return -1;
  }  
  int delay = DelayEstimatorProcess(&g_aecm.delay_estimator, Y_mag); // このdelayが4だったら4ブロック遅れ
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    delay = 0;  // 遅延が不明な場合は 0 と仮定する。
  }

  // 推定した遅延に合わせて遠端スペクトルを整列
  const uint16_t* X_mag_aligned = AlignedFarX(delay);
  if (X_mag_aligned == NULL) return -1;

  // エネルギーの履歴（log |X|, log |Ŷ|）を更新して VAD/閾値に反映
  CalcEnergies(X_mag_aligned, Y_mag_sum, S_mag);

  // 遠端エネルギーの変動に基づき NLMS のステップサイズ μ を算出
  int16_t mu = CalcStepSize();

  // 処理済みブロック数をインクリメント
  g_aecm.totCount++;

  // ここからチャネル推定アルゴリズム。NLMS 派生で、上で計算した
  // 可変ステップ長を用いて更新する。
  UpdateChannel(X_mag_aligned, 0 /*x_q*/, Y_mag, mu, S_mag);
  int16_t gGain = CalcSuppressionGain();

  // Wiener フィルタ H(k) を算出
  uint16_t* Y_mag_clean = Y_mag;  // |Y_clean(k)| proxy
  int16_t H_gain[PART_LEN1];
  int16_t numPosCoef = 0;
  double sum_gain = 0.0;
  for (int i = 0; i < PART_LEN1; i++) {
    int32_t tmp32no1 = S_mag[i] - g_aecm.sMagSmooth[i];
    g_aecm.sMagSmooth[i] += (int32_t)(((int64_t)tmp32no1 * 50) >> 8);

    int16_t zeros32 = NormW32(g_aecm.sMagSmooth[i]) + 1;
    int16_t zeros16 = NormW16(gGain) + 1;
    uint32_t sMagGained;
    int16_t resolutionDiff;
    if (zeros32 + zeros16 > 16) {
      sMagGained = UMUL_32_16((uint32_t)g_aecm.sMagSmooth[i], (uint16_t)gGain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
    } else {
      int16_t tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff = 14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      if (zeros32 > tmp16no1) {
        sMagGained = UMUL_32_16((uint32_t)g_aecm.sMagSmooth[i], gGain >> tmp16no1);
      } else {
        sMagGained = (g_aecm.sMagSmooth[i] >> tmp16no1) * gGain;
      }
    }

    zeros16 = NormW16(g_aecm.yMagSmooth[i]);
    int16_t y_mag_q_domain_diff = g_aecm.dfaCleanQDomain - g_aecm.dfaCleanQDomainOld;
    int16_t qDomainDiff;
    int16_t tmp16no1;
    int16_t tmp16no2;
    if (zeros16 < y_mag_q_domain_diff && g_aecm.yMagSmooth[i]) {
      tmp16no1 = g_aecm.yMagSmooth[i] * (1 << zeros16);
      qDomainDiff = zeros16 - y_mag_q_domain_diff;
      tmp16no2 = Y_mag_clean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = y_mag_q_domain_diff < 0
                     ? g_aecm.yMagSmooth[i] >> -y_mag_q_domain_diff
                     : g_aecm.yMagSmooth[i] * (1 << y_mag_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = Y_mag_clean[i];
    }
    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
    tmp16no2 += tmp16no1;
    zeros16 = NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      g_aecm.yMagSmooth[i] = WORD16_MAX;
    } else {
      g_aecm.yMagSmooth[i] = qDomainDiff < 0 ? tmp16no2 * (1 << -qDomainDiff)
                                             : tmp16no2 >> qDomainDiff;
    }

    if (sMagGained == 0) {
      H_gain[i] = ONE_Q14;
    } else if (g_aecm.yMagSmooth[i] == 0) {
      H_gain[i] = 0;
    } else {
      sMagGained += (uint32_t)(g_aecm.yMagSmooth[i] >> 1);
      uint32_t tmpU32 = DivU32U16(sMagGained, (uint16_t)g_aecm.yMagSmooth[i]);

      tmp32no1 = (int32_t)SHIFT_W32(tmpU32, resolutionDiff);
      if (tmp32no1 > ONE_Q14) {
        H_gain[i] = 0;
      } else if (tmp32no1 < 0) {
        H_gain[i] = ONE_Q14;
      } else {
        H_gain[i] = ONE_Q14 - (int16_t)tmp32no1;
        if (H_gain[i] < 0) {
          H_gain[i] = 0;
        }
      }
    }
    if (H_gain[i]) {
      numPosCoef++;
    }
  }
  if (g_bypass_wiener) {
    for (int i = 0; i < PART_LEN1; ++i) {
      H_gain[i] = ONE_Q14;
    }
    numPosCoef = PART_LEN1;
  }
  for (int i = 0; i < PART_LEN1; i++) {
    H_gain[i] = (int16_t)((H_gain[i] * H_gain[i]) >> 14);
  }

  const int kMinPrefBand = 4;
  const int kMaxPrefBand = 24;
  int32_t avgH32 = 0;
  for (int i = kMinPrefBand; i <= kMaxPrefBand; i++) {
    avgH32 += (int32_t)H_gain[i];
  }
  avgH32 /= (kMaxPrefBand - kMinPrefBand + 1);

  for (int i = kMaxPrefBand; i < PART_LEN1; i++) {
    if (H_gain[i] > (int16_t)avgH32) {
      H_gain[i] = (int16_t)avgH32;
    }
  }

  ComplexInt16 E_freq[PART_LEN2];
  for (int i = 0; i < PART_LEN1; i++) {
    if (H_gain[i] > NLP_COMP_HIGH) {
      H_gain[i] = ONE_Q14;
    } else if (H_gain[i] < NLP_COMP_LOW) {
      H_gain[i] = 0;
    }

    int16_t nlpGain = (numPosCoef < 3) ? 0 : ONE_Q14;
    if (g_bypass_nlp) {
      nlpGain = ONE_Q14;
    }

    if ((H_gain[i] == ONE_Q14) && (nlpGain == ONE_Q14)) {
      H_gain[i] = ONE_Q14;
    } else {
      H_gain[i] = (int16_t)((H_gain[i] * nlpGain) >> 14);
    }

    E_freq[i].real = (int16_t)(MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].real, H_gain[i], 14));
    E_freq[i].imag = (int16_t)(MUL_16_16_RSFT_WITH_ROUND(Y_freq[i].imag, H_gain[i], 14));
    double gain_normalized = static_cast<double>(H_gain[i]) / static_cast<double>(ONE_Q14);
    sum_gain += gain_normalized;
  }
  double avg_gain = sum_gain / PART_LEN1;
  double suppression_db;
  if (avg_gain <= 0.0) avg_gain = 0.0;
  double clamped_gain = avg_gain;
  const double kMinGain = 1e-3;
  if (clamped_gain < kMinGain) clamped_gain = kMinGain;
  suppression_db = 20.0 * log10(clamped_gain);

  {
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
  }

  int16_t fft[PART_LEN4 + 2];
  InverseFFTAndWindow(fft, E_freq, e_block);

  {
    static int dbg_ss_counter = 0;
    dbg_ss_counter++;
    if (dbg_ss_counter % 100 == 0) {
      fprintf(stderr, "[AECM] block=%d startupState=%d\n",
              dbg_ss_counter, (int)g_aecm.startupState);
    }
  }

  return 0;
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
void UpdateFarHistory(uint16_t* x_spectrum) {
  // 新しいバッファ位置を算出
  g_aecm.xHistoryPos++;
  if (g_aecm.xHistoryPos >= MAX_DELAY) {
    g_aecm.xHistoryPos = 0;
  }
  // Q-domain は固定Q=0のため保持不要
  // 遠端スペクトル用バッファを更新
  memcpy(&(g_aecm.xHistory[g_aecm.xHistoryPos * PART_LEN1]), x_spectrum, sizeof(uint16_t) * PART_LEN1);
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
  memcpy(g_aecm.hAdapt16, g_aecm.hStored, sizeof(int16_t) * PART_LEN1);
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
int InitCore() {
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

  static_assert(kRealFftOrder == PART_LEN_SHIFT,
                "FFT order と PART_LEN_SHIFT が不一致です");

  return 0;
}

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

// 近端・遠端・推定エコーのエネルギーを対数で算出し、
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
  uint32_t tmpAdapt = 0; // 適応チャネル経由の線形エコーエネルギー
  uint32_t tmpStored = 0; // 保存チャネル経由の線形エコーエネルギー
  uint32_t tmpFar = 0; // 遠端信号の線形エネルギー

  int16_t tmp16; // 閾値計算などで使う一時的な16ビット値
  int16_t increase_max_shifts = 4; // 上限フィルタの増加時シフト量
  int16_t decrease_max_shifts = 11; // 上限フィルタの減少時シフト量
  int16_t increase_min_shifts = 11; // 下限フィルタの増加時シフト量
  int16_t decrease_min_shifts = 3; // 下限フィルタの減少時シフト量

  // 近端エネルギーの対数を求め、バッファへ格納

  // バッファをシフト
  memmove(g_aecm.nearLogEnergy + 1, g_aecm.nearLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));

  // 近端振幅積分の対数 (nearEner)
  g_aecm.nearLogEnergy[0] = LogOfEnergyInQ8(Y_energy, g_aecm.dfaNoisyQDomain);

  CalcLinearEnergiesC(X_mag, S_mag, &tmpFar, &tmpAdapt, &tmpStored);

  // ログ履歴バッファをシフト
  memmove(g_aecm.echoAdaptLogEnergy + 1, g_aecm.echoAdaptLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));
  memmove(g_aecm.echoStoredLogEnergy + 1, g_aecm.echoStoredLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));

  // 遅延後遠端エネルギーの対数
  g_aecm.farLogEnergy = LogOfEnergyInQ8(tmpFar, 0);

  // 適応チャネル経由推定エコーの対数エネルギー
  g_aecm.echoAdaptLogEnergy[0] = LogOfEnergyInQ8(tmpAdapt, RESOLUTION_CHANNEL16);

  // 保存チャネル経由推定エコーの対数エネルギー
  g_aecm.echoStoredLogEnergy[0] = LogOfEnergyInQ8(tmpStored, RESOLUTION_CHANNEL16);

  // 遠端エネルギー関連の閾値（最小・最大・VAD・MSE）を更新
  if (g_aecm.farLogEnergy > FAR_ENERGY_MIN) {
    if (g_aecm.startupState == 0) {
      increase_max_shifts = 2;
      decrease_min_shifts = 2;
      increase_min_shifts = 8;
    }

    g_aecm.farEnergyMin = AsymFilt(g_aecm.farEnergyMin, g_aecm.farLogEnergy, increase_min_shifts, decrease_min_shifts);
    g_aecm.farEnergyMax = AsymFilt(g_aecm.farEnergyMax, g_aecm.farLogEnergy, increase_max_shifts, decrease_max_shifts);
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
        g_aecm.farEnergyVAD += (g_aecm.farLogEnergy + tmp16 - g_aecm.farEnergyVAD) >> 6;
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
      tmp16no1 = zerosDfa - 2 + g_aecm.dfaNoisyQDomain - RESOLUTION_CHANNEL32 - x_q + shiftChFar;
      if (zerosNum > tmp16no1 + 1) {
        xfaQ = tmp16no1;
        yMagQ = zerosDfa - 2;
      } else {
        xfaQ = zerosNum - 2;
        yMagQ = RESOLUTION_CHANNEL32 + x_q - g_aecm.dfaNoisyQDomain - shiftChFar + xfaQ;
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
            tmp32no2 = (int32_t)UMUL_32_16(tmp32no1, X_mag[i]);
          } else {
            tmp32no2 = -(int32_t)UMUL_32_16(-tmp32no1, X_mag[i]);
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
        tmp32no1 = ((int32_t)g_aecm.echoStoredLogEnergy[i] - (int32_t)g_aecm.nearLogEnergy[i]);
        tmp32no2 = ABS_W32(tmp32no1);
        mseStored += tmp32no2;

        tmp32no1 = ((int32_t)g_aecm.echoAdaptLogEnergy[i] - (int32_t)g_aecm.nearLogEnergy[i]);
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
    tmp16no1 = (g_aecm.nearLogEnergy[0] - g_aecm.echoStoredLogEnergy[0] - ENERGY_DEV_OFFSET);
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
        tmp16no1 = (int16_t)DivW32W16(tmp32no1, (ENERGY_DEV_TOL - SUPGAIN_EPC_DT));
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
    memcpy(g_aecm.xFrameBuf + g_aecm.xBufWritePos, x_frame + writePos, sizeof(int16_t) * writeLen);
    g_aecm.xBufWritePos = 0;
    writePos = writeLen;
    writeLen = xLen - writeLen;
  }

  memcpy(g_aecm.xFrameBuf + g_aecm.xBufWritePos, x_frame + writePos, sizeof(int16_t) * writeLen);
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
    memcpy(x_frame + readPos, g_aecm.xFrameBuf + g_aecm.xBufReadPos, sizeof(int16_t) * readLen);
    g_aecm.xBufReadPos = 0;
    readPos = readLen;
    readLen = xLen - readLen;
  }
  memcpy(x_frame + readPos, g_aecm.xFrameBuf + g_aecm.xBufReadPos, sizeof(int16_t) * readLen);
  g_aecm.xBufReadPos += readLen;
}


static int EstimateBufDelay() {
  short nSampFar = static_cast<short>(available_read(&g_mobile.farendBuf));
  short nSampSndCard = kFixedMsInSndCardBuf * kSamplesPerMs16k;
  short delayNew = nSampSndCard - nSampFar;

  if (delayNew < FRAME_LEN) {
    MoveReadPtr(&g_mobile.farendBuf, FRAME_LEN);
    delayNew += FRAME_LEN;
  }

  g_mobile.filtDelay =
      static_cast<short>(MAX(0, (8 * g_mobile.filtDelay + 2 * delayNew) / 10));

  short diff = g_mobile.filtDelay - g_mobile.knownDelay;
  if (diff > 224) {
    if (g_mobile.lastDelayDiff < 96) {
      g_mobile.timeForDelayChange = 0;
    } else {
      g_mobile.timeForDelayChange++;
    }
  } else if (diff < 96 && g_mobile.knownDelay > 0) {
    if (g_mobile.lastDelayDiff > 224) {
      g_mobile.timeForDelayChange = 0;
    } else {
      g_mobile.timeForDelayChange++;
    }
  } else {
    g_mobile.timeForDelayChange = 0;
  }
  g_mobile.lastDelayDiff = diff;

  if (g_mobile.timeForDelayChange > 25) {
    g_mobile.knownDelay = MAX(static_cast<int>(g_mobile.filtDelay) - (2 * FRAME_LEN), 0);
  }
  return 0;
}

int32_t Init() {
  memset(&g_mobile, 0, sizeof(g_mobile));
  InitBufferWith(&g_mobile.farendBuf, g_mobile.farendBufData, kBufSizeSamples, sizeof(int16_t));

  if (InitCore() == -1) return -1;

  InitBuffer(&g_mobile.farendBuf);

  g_mobile.initFlag = kInitCheck;
  g_mobile.bufSizeStart = kStartupFrames;
  g_mobile.ECstartup = 1;
  g_mobile.filtDelay = 0;
  g_mobile.timeForDelayChange = 0;
  g_mobile.knownDelay = 0;
  g_mobile.lastDelayDiff = 0;

  memset(g_mobile.farendOld, 0, sizeof(g_mobile.farendOld));

  return 0;
}

int32_t BufferFarend(const int16_t* farend) {
  if (farend == NULL) return -1;
  if (g_mobile.initFlag != kInitCheck) return -2;

  WriteBuffer(&g_mobile.farendBuf, farend, FRAME_LEN);
  return 0;
}

int32_t Process(const int16_t* nearend, int16_t* out) {
  if (nearend == NULL) return -1;
  if (out == NULL) return -2;
  if (g_mobile.initFlag != kInitCheck) return -3;

  const size_t nrOfSamples = FRAME_LEN;
  const size_t nFrames = nrOfSamples / FRAME_LEN;  // 64/64=1（16kHz固定）

  if (g_mobile.ECstartup) {
    if (out != nearend) {
      memcpy(out, nearend, sizeof(short) * nrOfSamples);
    }

    short nmbrOfFilledBuffers =
        static_cast<short>(available_read(&g_mobile.farendBuf) / FRAME_LEN);
    if (nmbrOfFilledBuffers >= g_mobile.bufSizeStart) {
      if (nmbrOfFilledBuffers > g_mobile.bufSizeStart) {
        MoveReadPtr(&g_mobile.farendBuf,
                    static_cast<int>(available_read(&g_mobile.farendBuf)) -
                        static_cast<int>(g_mobile.bufSizeStart) * FRAME_LEN);
      }
      g_mobile.ECstartup = 0;
    }
  } else {
    for (size_t i = 0; i < nFrames; i++) {
      int16_t farend[FRAME_LEN];
      int16_t* farend_ptr = nullptr;

      short nmbrOfFilledBuffers =
          static_cast<short>(available_read(&g_mobile.farendBuf) / FRAME_LEN);

      if (nmbrOfFilledBuffers > 0) {
        ReadBuffer(&g_mobile.farendBuf, reinterpret_cast<void**>(&farend_ptr),
                   farend, FRAME_LEN);
        memcpy(g_mobile.farendOld, farend_ptr, FRAME_LEN * sizeof(short));
      } else {
        memcpy(farend, g_mobile.farendOld, FRAME_LEN * sizeof(short));
        farend_ptr = farend;
      }

      if (i == nFrames - 1) {
        EstimateBufDelay();
      }

      if (ProcessFrame(farend_ptr, &nearend[FRAME_LEN * i], &out[FRAME_LEN * i]) == -1) {
        return -1;
      }
    }
  }

  return 0;
}

void SetBypassWiener(int enable) { SetBypassWienerInternal(enable); }

void SetBypassNlp(int enable) { SetBypassNlpInternal(enable); }
