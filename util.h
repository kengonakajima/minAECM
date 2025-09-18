#ifndef UTIL_H_
#define UTIL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ComplexInt16 {
  int16_t real;
  int16_t imag;
} ComplexInt16;

#define WORD16_MAX 32767 // 16ビット整数の最大値
#define WORD16_MIN -32768 // 16ビット整数の最小値
#define WORD32_MAX (int32_t)0x7fffffff // 32ビット整数の最大値
#define WORD32_MIN (int32_t)0x80000000 // 32ビット整数の最小値

#define MIN(A, B) ((A) < (B) ? (A) : (B)) // 小さい方を返す
#define MAX(A, B) ((A) > (B) ? (A) : (B)) // 大きい方を返す

#define ABS_W16(a) (((int16_t)(a) >= 0) ? ((int16_t)(a)) : -((int16_t)(a))) // 16ビットの絶対値
#define ABS_W32(a) (((int32_t)(a) >= 0) ? ((int32_t)(a)) : -((int32_t)(a))) // 32ビットの絶対値

#define UMUL_32_16(a, b) ((uint32_t)((uint32_t)(a) * (uint16_t)(b))) // 32×16ビットの符号なし乗算
#define MUL_16_U16(a, b) ((int32_t)(int16_t)(a) * (uint16_t)(b)) // 符号付き16×符号なし16の乗算
#define MUL_16_16(a, b) ((int32_t)(((int16_t)(a)) * ((int16_t)(b)))) // 符号付き16ビット乗算
#define MUL_16_16_RSFT(a, b, c) (MUL_16_16(a, b) >> (c)) // 乗算後右シフト
#define MUL_16_16_RSFT_WITH_ROUND(a, b, c) ((MUL_16_16(a, b) + ((int32_t)1 << ((c) - 1))) >> (c)) // 乗算後右シフト丸め

#define SAT(a, b, c) ((b) > (a) ? (a) : (b) < (c) ? (c) : (b)) // 上下限クリップ
#define SHIFT_W32(x, c) ((c) >= 0 ? (x) * (1 << (c)) : (x) >> -(c)) // 32ビットの可変シフト

enum Wrap { SAME_WRAP, DIFF_WRAP };

typedef struct RingBuffer {
  size_t read_pos; // 読み出し位置インデックス
  size_t write_pos; // 書き込み位置インデックス
  size_t element_count; // 要素総数
  size_t element_size; // 要素サイズ（バイト）
  enum Wrap rw_wrap; // 読み書き位置の巻き戻り状態
  char* data; // 格納先バッファ先頭
} RingBuffer;

int CountLeadingZeros32(uint32_t n); // 32ビット値の先頭ゼロ数計算
int CountLeadingZeros64(uint64_t n); // 64ビット値の先頭ゼロ数計算
int16_t SatW32ToW16(int32_t value32); // 32ビットを16ビットへ飽和変換
int32_t AddSatW32(int32_t a, int32_t b); // 32ビット飽和加算
int32_t SubSatW32(int32_t a, int32_t b); // 32ビット飽和減算
int16_t AddSatW16(int16_t a, int16_t b); // 16ビット飽和加算
int16_t SubSatW16(int16_t a, int16_t b); // 16ビット飽和減算
int16_t NormW32(int32_t a); // 32ビット正規化シフト量取得
int16_t NormU32(uint32_t a); // 32ビット符号なし正規化シフト量
int16_t NormW16(int16_t a); // 16ビット正規化シフト量取得
int16_t GetSizeInBits(uint32_t n); // ビット幅取得
int32_t MulAccumW16(int16_t a, int16_t b, int32_t c); // 16ビット乗算累積
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length); // 16ビット最大絶対値
#define MaxAbsValueW16 MaxAbsValueW16C // 最大絶対値関数エイリアス
uint32_t DivU32U16(uint32_t num, uint16_t den); // 符号なし32÷16除算
int32_t DivW32W16(int32_t num, int16_t den); // 符号付き32÷16除算
int32_t SqrtFloor(int32_t value); // 平方根の床値
int16_t ExtractFractionPart(uint32_t a, int zeros);
int16_t LogOfEnergyInQ8(uint32_t energy, int q_domain);

void InitBuffer(RingBuffer* handle); // リングバッファ初期化
void InitBufferWith(RingBuffer* handle, void* backing, size_t element_count, size_t element_size); // 外部メモリ付き初期化
size_t ReadBuffer(RingBuffer* handle, void** data_ptr, void* data, size_t element_count); // リングバッファ読み出し
size_t WriteBuffer(RingBuffer* handle, const void* data, size_t element_count); // リングバッファ書き込み
int MoveReadPtr(RingBuffer* handle, int element_count); // 読み位置移動
size_t available_read(const RingBuffer* handle); // 読み可能要素数取得
size_t available_write(const RingBuffer* handle); // 書き可能要素数取得

// 固定長 2^7 (=128) ポイント FFT を前提にした実数 FFT ルーチン
enum { kRealFftOrder = 7 };

int RealForwardFFT(const int16_t* real_data_in, int16_t* complex_data_out); // 実数入力の前方FFT
int RealInverseFFT(const int16_t* complex_data_in, int16_t* real_data_out); // 実数出力の逆FFT

int ComplexFFT(int16_t vector[], int stages, int mode); // 複素数前方FFT
int ComplexIFFT(int16_t vector[], int stages, int mode); // 複素数逆FFT
void ComplexBitReverse(int16_t* __restrict complex_data, int stages); // ビット反転並べ替え

void InverseFFTAndWindow(int16_t* fft,
                         ComplexInt16* efw,
                         int part_len,
                         int part_len2,
                         const int16_t* sqrt_hanning,
                         int16_t* current_block,
                         int16_t* overlap_block);

void WindowAndFFT(int16_t* fft,
                  const int16_t* time_signal,
                  ComplexInt16* freq_signal,
                  int part_len,
                  const int16_t* sqrt_hanning);

#ifdef __cplusplus
}
#endif

#endif  // UTIL_H_
