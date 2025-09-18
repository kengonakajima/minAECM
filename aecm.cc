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


int g_firstVAD; // VAD 初回検出フラグ
uint16_t g_xHistory[PART_LEN1 * MAX_DELAY]; // 遠端スペクトル履歴（遅延候補ごと）
int g_xHistoryPos; // 遠端スペクトル履歴の書き込みインデックス


uint32_t g_totCount; // 処理済みブロック数のカウンタ

int16_t g_dfaCleanQDomain; // クリーン成分の Q-domain 推定値
int16_t g_dfaCleanQDomainOld; // 上記の1ブロック前の値
int16_t g_dfaNoisyQDomain; // 雑音成分の Q-domain 推定値
int16_t g_dfaNoisyQDomainOld; // 雑音 Q-domain の1ブロック前の値


int16_t g_nearLogEnergy[MAX_LOG_LEN]; // 近端信号のログエネルギー履歴
int16_t g_farLogEnergy; // 遠端信号のログエネルギー最新値
int16_t g_echoAdaptLogEnergy[MAX_LOG_LEN]; // 適応エコーパスによるログエネルギー履歴
int16_t g_echoStoredLogEnergy[MAX_LOG_LEN]; // 保存エコーパスによるログエネルギー履歴


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
int16_t g_farEnergyMSE; // MSE 判定用遠端エネルギー
int g_currentVADValue; // 近端 VAD の現在フラグ
int16_t g_vadUpdateCount; // VAD 関連の更新カウンタ

int16_t g_startupState; // 起動フェーズの状態
int16_t g_mseChannelCount; // MSE 判定でのチャネル更新回数
int16_t g_supGain; // 現在の抑圧ゲイン（Q8）
int16_t g_supGainOld; // 直前の抑圧ゲイン（Q8）


// 先行宣言（翻訳単位内のみで使用）。
void UpdateFarHistory(uint16_t* x_spectrum);
void CalcLinearEnergies(const uint16_t* X_mag,
                        int32_t* S_mag,
                        uint32_t* X_energy,
                        uint32_t* S_energy_adapt,
                        uint32_t* S_energy_stored);
void UpdateChannel(const uint16_t* X_mag,
                   const uint16_t* const Y_mag,
                   int16_t mu,
                   int32_t* S_mag);
int16_t LogOfEnergyInQ8(uint32_t energy, int q_domain);
int16_t AsymFilt(const int16_t filtOld,
                 const int16_t inVal,
                 const int16_t stepSizePos,
                 const int16_t stepSizeNeg);
// 直接ブロック処理を行うため、旧BlockFramer相当のバッファは撤廃済み。

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
    output[i] = (int16_t)SAT(WORD16_MAX, tmp32no1 + g_eOverlapBuf[i], WORD16_MIN);

    tmp32no1 = (ifft_out[PART_LEN + i] * kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = SHIFT_W32(tmp32no1, outCFFT);
    g_eOverlapBuf[i] = (int16_t)SAT(WORD16_MAX, tmp32no1, WORD16_MIN);
  }

  // 現ブロックの値を過去位置へコピーし、
  // （`g_eOverlapBuf` のシフトは別処理で行う）
  memcpy(g_xBuf, g_xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(g_yBuf, g_yBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
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

  if (g_startupState < 2) {
    g_startupState = (g_totCount >= CONV_LEN) + (g_totCount >= CONV_LEN2);
  }

  // 近端/遠端の時間領域フレームをバッファへ蓄える
  memcpy(g_xBuf + PART_LEN, x_block, sizeof(int16_t) * PART_LEN);
  memcpy(g_yBuf + PART_LEN, y_block, sizeof(int16_t) * PART_LEN);

  // 遠端信号 X(k) = FFT{x(n)} を算出し（|X| と Σ|X| を求める）
  uint32_t X_mag_sum = 0;
  TimeToFrequencyDomain(g_xBuf, Y_freq, X_mag, &X_mag_sum);

  // 近端信号 Y(k) = FFT{y(n)} を算出し（|Y| と Σ|Y| を求める）
  uint32_t Y_mag_sum = 0;
  TimeToFrequencyDomain(g_yBuf, Y_freq, Y_mag, &Y_mag_sum);
  g_dfaNoisyQDomainOld = g_dfaNoisyQDomain;
  g_dfaNoisyQDomain = 0;

  g_dfaCleanQDomainOld = g_dfaNoisyQDomainOld;
  g_dfaCleanQDomain = g_dfaNoisyQDomain;

  // 遠端スペクトル履歴を更新し、2値スペクトル照合で遅延を推定
  UpdateFarHistory(X_mag);
  if (AddFarSpectrum(X_mag) == -1) {
    return -1;
  }  
  int delay = DelayEstimatorProcess(Y_mag); // このdelayが4だったら4ブロック遅れ
  if (delay == -1) {
    return -1;
  } else if (delay == -2) {
    delay = 0;  // 遅延が不明な場合は 0 と仮定する。
  }

  // 推定した遅延に合わせて遠端スペクトルを整列
  int buffer_position = g_xHistoryPos - delay;
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  const uint16_t* X_mag_aligned =
      &(g_xHistory[buffer_position * PART_LEN1]);

  // エネルギーの履歴（log |X|, log |Ŷ|）を更新して VAD/閾値に反映
  {
    uint32_t tmpFar = 0;
    uint32_t tmpAdapt = 0;
    uint32_t tmpStored = 0;
    int16_t tmp16;
    int16_t increase_max_shifts = 4;
    int16_t decrease_max_shifts = 11;
    int16_t increase_min_shifts = 11;
    int16_t decrease_min_shifts = 3;

    memmove(g_nearLogEnergy + 1, g_nearLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));
    g_nearLogEnergy[0] = LogOfEnergyInQ8(Y_mag_sum, g_dfaNoisyQDomain);

    CalcLinearEnergies(X_mag_aligned, S_mag, &tmpFar, &tmpAdapt, &tmpStored);

    memmove(g_echoAdaptLogEnergy + 1, g_echoAdaptLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));
    memmove(g_echoStoredLogEnergy + 1, g_echoStoredLogEnergy, sizeof(int16_t) * (MAX_LOG_LEN - 1));

    g_farLogEnergy = LogOfEnergyInQ8(tmpFar, 0);
    g_echoAdaptLogEnergy[0] = LogOfEnergyInQ8(tmpAdapt, RESOLUTION_CHANNEL16);
    g_echoStoredLogEnergy[0] = LogOfEnergyInQ8(tmpStored, RESOLUTION_CHANNEL16);

    if (g_farLogEnergy > FAR_ENERGY_MIN) {
      if (g_startupState == 0) {
        increase_max_shifts = 2;
        decrease_min_shifts = 2;
        increase_min_shifts = 8;
      }

      g_farEnergyMin = AsymFilt(g_farEnergyMin, g_farLogEnergy, increase_min_shifts, decrease_min_shifts);
      g_farEnergyMax = AsymFilt(g_farEnergyMax, g_farLogEnergy, increase_max_shifts, decrease_max_shifts);
      g_farEnergyMaxMin = g_farEnergyMax - g_farEnergyMin;

      tmp16 = 2560 - g_farEnergyMin;
      if (tmp16 > 0) {
        tmp16 = static_cast<int16_t>((tmp16 * FAR_ENERGY_VAD_REGION) >> 9);
      } else {
        tmp16 = 0;
      }
      tmp16 += FAR_ENERGY_VAD_REGION;

      if ((g_startupState == 0) | (g_vadUpdateCount > 1024)) {
        g_farEnergyVAD = g_farEnergyMin + tmp16;
      } else {
        if (g_farEnergyVAD > g_farLogEnergy) {
          g_farEnergyVAD += (g_farLogEnergy + tmp16 - g_farEnergyVAD) >> 6;
          g_vadUpdateCount = 0;
        } else {
          g_vadUpdateCount++;
        }
      }
      g_farEnergyMSE = g_farEnergyVAD + (1 << 8);
    }

    if (g_farLogEnergy > g_farEnergyVAD) {
      if ((g_startupState == 0) | (g_farEnergyMaxMin > FAR_ENERGY_DIFF)) {
        g_currentVADValue = 1;
      }
    } else {
      g_currentVADValue = 0;
    }
    if (g_currentVADValue && g_firstVAD) {
      g_firstVAD = 0;
      if (g_echoAdaptLogEnergy[0] > g_nearLogEnergy[0]) {
        for (int i = 0; i < PART_LEN1; i++) {
          g_HAdapt16[i] >>= 3;
        }
        g_echoAdaptLogEnergy[0] -= (3 << 8);
        g_firstVAD = 1;
      }
    }
  }

  // 遠端エネルギーの変動に基づき NLMS のステップサイズ μ を算出
  int16_t mu = MU_MAX;
  if (!g_currentVADValue) {
    mu = 0;
  } else if (g_startupState > 0) {
    if (g_farEnergyMin >= g_farEnergyMax) {
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

  // ここからチャネル推定アルゴリズム。NLMS 派生で、上で計算した
  // 可変ステップ長を用いて H_adapt を更新する。
  UpdateChannel(X_mag_aligned, Y_mag, mu, S_mag);
  int16_t G_gain;
  {
    int32_t tmp32no1;
    int16_t supGain = SUPGAIN_DEFAULT;
    int16_t tmp16no1;
    int16_t dE = 0;

    if (!g_currentVADValue) {
      supGain = 0;
    } else {
      tmp16no1 = (g_nearLogEnergy[0] - g_echoStoredLogEnergy[0] - ENERGY_DEV_OFFSET);
      dE = ABS_W16(tmp16no1);

      if (dE < ENERGY_DEV_TOL) {
        if (dE < SUPGAIN_EPC_DT) {
          const int diffAB = SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
          tmp32no1 = diffAB * dE;
          tmp32no1 += (SUPGAIN_EPC_DT >> 1);
          tmp16no1 = (int16_t)DivW32W16(tmp32no1, SUPGAIN_EPC_DT);
          supGain = SUPGAIN_ERROR_PARAM_A - tmp16no1;
        } else {
          const int diffBD = SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;
          tmp32no1 = diffBD * (ENERGY_DEV_TOL - dE);
          tmp32no1 += ((ENERGY_DEV_TOL - SUPGAIN_EPC_DT) >> 1);
          tmp16no1 = (int16_t)DivW32W16(tmp32no1, (ENERGY_DEV_TOL - SUPGAIN_EPC_DT));
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

    G_gain = g_supGain;
    // 抑圧ゲイン更新ここまで
  }

  // Wiener/NLP 用の抑圧マスク G(k) を算出
  uint16_t* Y_mag_clean = Y_mag;  // |Y_clean(k)| proxy
  int16_t G_mask[PART_LEN1];
  int16_t numPosCoef = 0;
  double sum_gain = 0.0;
  for (int i = 0; i < PART_LEN1; i++) {
    int32_t tmp32no1 = S_mag[i] - g_sMagSmooth[i];
    g_sMagSmooth[i] += (int32_t)(((int64_t)tmp32no1 * 50) >> 8);

    int16_t zeros32 = NormW32(g_sMagSmooth[i]) + 1;
    int16_t zeros16 = NormW16(G_gain) + 1;
    uint32_t S_magGained;
    int16_t resolutionDiff;
    if (zeros32 + zeros16 > 16) {
      S_magGained = UMUL_32_16((uint32_t)g_sMagSmooth[i], (uint16_t)G_gain);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
    } else {
      int16_t tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff = 14 + tmp16no1 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      if (zeros32 > tmp16no1) {
        S_magGained = UMUL_32_16((uint32_t)g_sMagSmooth[i], G_gain >> tmp16no1);
      } else {
        S_magGained = (g_sMagSmooth[i] >> tmp16no1) * G_gain;
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
      tmp16no2 = Y_mag_clean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = y_mag_q_domain_diff < 0
                     ? g_yMagSmooth[i] >> -y_mag_q_domain_diff
                     : g_yMagSmooth[i] * (1 << y_mag_q_domain_diff);
      qDomainDiff = 0;
      tmp16no2 = Y_mag_clean[i];
    }
    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
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

      tmp32no1 = (int32_t)SHIFT_W32(tmpU32, resolutionDiff);
      if (tmp32no1 > ONE_Q14) {
        G_mask[i] = 0;
      } else if (tmp32no1 < 0) {
        G_mask[i] = ONE_Q14;
      } else {
        G_mask[i] = ONE_Q14 - (int16_t)tmp32no1;
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
              dbg_ss_counter, (int)g_startupState);
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
  g_xHistoryPos++;
  if (g_xHistoryPos >= MAX_DELAY) {
    g_xHistoryPos = 0;
  }
  // Q-domain は固定Q=0のため保持不要
  // 遠端スペクトル用バッファを更新
  memcpy(&(g_xHistory[g_xHistoryPos * PART_LEN1]), x_spectrum, sizeof(uint16_t) * PART_LEN1);
}


void InitEchoPathCore(const int16_t* echo_path) {
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

void CalcLinearEnergies(const uint16_t* X_mag,
                         int32_t* S_mag,
                         uint32_t* X_energy,
                         uint32_t* S_energy_adapt,
                         uint32_t* S_energy_stored) {
  // 遅延後の遠端信号と推定エコーのエネルギーを取得（保存/適応チャネル双方）
  // 
  for (int i = 0; i < PART_LEN1; i++) {
    S_mag[i] = MUL_16_U16(g_HStored[i], X_mag[i]);
    (*X_energy) += (uint32_t)(X_mag[i]);
    *S_energy_adapt += g_HAdapt16[i] * X_mag[i];
    (*S_energy_stored) += (uint32_t)S_mag[i];
  }
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


int Init() {
  // 16kHz 固定
  memset(g_xBuf, 0, sizeof(g_xBuf));
  memset(g_yBuf, 0, sizeof(g_yBuf));
  memset(g_eOverlapBuf, 0, sizeof(g_eOverlapBuf));

  g_totCount = 0;

  if (InitDelayEstimatorFarend() != 0) {
    return -1;
  }
  if (InitDelayEstimator() != 0) {
    return -1;
  }
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
  InitEchoPathCore(kChannelStored16kHz);

  memset(g_sMagSmooth, 0, sizeof(g_sMagSmooth));
  memset(g_yMagSmooth, 0, sizeof(g_yMagSmooth));

  g_farEnergyMin = WORD16_MAX;
  g_farEnergyMax = WORD16_MIN;
  g_farEnergyMaxMin = 0;
  g_farEnergyVAD = FAR_ENERGY_MIN;  // 開始直後の誤検出（音声とみなさない）を防ぐ
                                        // 
  g_farEnergyMSE = 0;
  g_currentVADValue = 0;
  g_vadUpdateCount = 0;
  g_firstVAD = 1;

  g_startupState = 0;
  g_supGain = SUPGAIN_DEFAULT;
  g_supGainOld = SUPGAIN_DEFAULT;

  // コンパイル時に前提条件を static_assert で確認
  // アセンブリ実装が依存するため、修正時は該当ファイルを要確認。
  static_assert(PART_LEN % 16 == 0, "PART_LEN is not a multiple of 16");

  static_assert(kRealFftOrder == PART_LEN_SHIFT,
                "FFT order と PART_LEN_SHIFT が不一致です");

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

// NLMS によるチャネル推定と保存判定。
// X_mag: 遠端振幅スペクトル(Q0)、Y_mag: 近端振幅スペクトル(Q0)、
// mu: 上記で算出したシフト量、S_mag: 推定エコー（Q=RESOLUTION_CHANNEL16）。
void UpdateChannel(const uint16_t* X_mag,
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
      zerosCh = NormU32(g_HAdapt32[i]);
      zerosFar = NormU32((uint32_t)X_mag[i]);
      if (zerosCh + zerosFar > 31) {
        // 乗算しても安全な状態
        tmpU32no1 = UMUL_32_16(g_HAdapt32[i], X_mag[i]);
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
      tmp16no1 = zerosDfa - 2 + g_dfaNoisyQDomain - RESOLUTION_CHANNEL32 + shiftChFar;
      if (zerosNum > tmp16no1 + 1) {
        xfaQ = tmp16no1;
        yMagQ = zerosDfa - 2;
      } else {
        xfaQ = zerosNum - 2;
        yMagQ = RESOLUTION_CHANNEL32 - g_dfaNoisyQDomain - shiftChFar + xfaQ;
      }
      // 同じ Q ドメインに揃えて加算
      tmpU32no1 = SHIFT_W32(tmpU32no1, xfaQ);
      tmpU32no2 = SHIFT_W32((uint32_t)Y_mag[i], yMagQ);
      tmp32no1 = (int32_t)tmpU32no2 - (int32_t)tmpU32no1;
      zerosNum = NormW32(tmp32no1);
      if (tmp32no1 && (X_mag[i] > CHANNEL_VAD)) {
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
        g_HAdapt32[i] = AddSatW32(g_HAdapt32[i], tmp32no2);
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
  if ((g_startupState == 0) & (g_currentVADValue)) {
    // 起動中は毎ブロックチャネルを保存し、
    // 推定エコーも再計算する
    StoreAdaptiveChannel(X_mag, S_mag);
  } else {
    if (g_farLogEnergy < g_farEnergyMSE) {
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
        tmp32no1 = ((int32_t)g_echoStoredLogEnergy[i] - (int32_t)g_nearLogEnergy[i]);
        tmp32no2 = ABS_W32(tmp32no1);
        mseStored += tmp32no2;

        tmp32no1 = ((int32_t)g_echoAdaptLogEnergy[i] - (int32_t)g_nearLogEnergy[i]);
        tmp32no2 = ABS_W32(tmp32no1);
        mseAdapt += tmp32no2;
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
