// AECM_DYNAMIC_Q を無効化（教育用に定数Qで簡略化） 


// アルゴリズム関連の定数 
// 教育用にフレームとパーティションを一致させる: 64サンプル固定
#define BLOCK_LEN 64  // ブロック長（PART_LEN と同じ） 

#define PART_LEN 64      // パーティション長。ブロック単位
#define PART_LEN_SHIFT 7 /* PART_LEN*2 を表すビットシフト量 */

#define PART_LEN1 (PART_LEN + 1)  /* FFT のユニークな係数数 */
#define PART_LEN2 (PART_LEN << 1) /* パーティション長の 2 倍 */
#define PART_LEN4 (PART_LEN << 2) /* パーティション長の 4 倍 */
#define FAR_BUF_LEN PART_LEN4     /* 遠端バッファの長さ */
#define MAX_DELAY 100 // 推定可能な最大の遅延。単位はブロック


/* アプリ統合向けの公開定数 */
#define SAMPLE_RATE_HZ 16000


/* カウンタ関連の定数 */
#define CONV_LEN 512              /* 起動時に用いる収束ブロック数 */
#define CONV_LEN2 (CONV_LEN << 1) /* 起動時に使用する 2 倍長 */

/* エネルギー関連の定数 */
#define MAX_LOG_LEN 64            /* 対数エネルギー履歴の長さ */
#define FAR_ENERGY_MIN 1025       /* 遠端エネルギーの下限（少なくとも 2） */
#define FAR_ENERGY_DIFF 929       /* 最大値と最小値の許容差 */
#define ENERGY_DEV_OFFSET 0       /* エネルギー誤差のオフセット（Q8） */
#define ENERGY_DEV_TOL 400        /* エネルギー推定の許容誤差（Q8） */
#define FAR_ENERGY_VAD_REGION 230 /* 遠端 VAD の許容範囲 */

/* チャネル関連の定数 */
#define MIN_MSE_COUNT 20 /* チャネル比較に必要な遠端エネルギー十分な連続ブロック数の最小値 */
#define MIN_MSE_DIFF 29  /* 適応/保存チャネルを比較して保存更新を許可する比率（Q-MSE_RESOLUTION で 0.8） */
#define MSE_RESOLUTION 5 /* MSE パラメータの分解能 */
#define RESOLUTION_CHANNEL16 12 /* 16 ビットチャネル係数の Q 解像度 */
#define RESOLUTION_CHANNEL32 28 /* 32 ビットチャネル係数の Q 解像度 */
#define CHANNEL_VAD 16          /* チャネル更新に必要な帯域内の最小エネルギー */

/* 抑圧ゲイン関連の定数（Q-RESOLUTION_SUPGAIN で表現） */
#define RESOLUTION_SUPGAIN 8 /* 抑圧ゲインの Q 解像度 */
#define SUPGAIN_DEFAULT (1 << RESOLUTION_SUPGAIN) /* デフォルト値 */
#define SUPGAIN_ERROR_PARAM_A 3072            /* 推定誤差制御パラメータA（最大ゲイン、Q8 で約 8） */
#define SUPGAIN_ERROR_PARAM_B 1536            /* 推定誤差制御パラメータB（減衰前のゲイン） */
#define SUPGAIN_ERROR_PARAM_D SUPGAIN_DEFAULT /* 推定誤差制御パラメータD（デフォルトと同値、Q8 で 1） */
#define SUPGAIN_EPC_DT 200 /* SUPGAIN_ERROR_PARAM_C と ENERGY_DEV_TOL の積 */

#define ONE_Q14 (1 << 14)

/* NLP 関連定数 */
#define NLP_COMP_LOW 3277     /* Q14 で 0.2 */
#define NLP_COMP_HIGH ONE_Q14 /* Q14 で 1.0 */
