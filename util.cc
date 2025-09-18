// FFT・リングバッファ周りの軽量ユーティリティ実装
#include "util.h"

#include <limits.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static const int16_t kSinTable1024[] = {
    0,      201,    402,    603,    804,    1005,   1206,   1406,   1607,
    1808,   2009,   2209,   2410,   2610,   2811,   3011,   3211,   3411,
    3611,   3811,   4011,   4210,   4409,   4608,   4807,   5006,   5205,
    5403,   5601,   5799,   5997,   6195,   6392,   6589,   6786,   6982,
    7179,   7375,   7571,   7766,   7961,   8156,   8351,   8545,   8739,
    8932,   9126,   9319,   9511,   9703,   9895,   10087,  10278,  10469,
    10659,  10849,  11038,  11227,  11416,  11604,  11792,  11980,  12166,
    12353,  12539,  12724,  12909,  13094,  13278,  13462,  13645,  13827,
    14009,  14191,  14372,  14552,  14732,  14911,  15090,  15268,  15446,
    15623,  15799,  15975,  16150,  16325,  16499,  16672,  16845,  17017,
    17189,  17360,  17530,  17699,  17868,  18036,  18204,  18371,  18537,
    18702,  18867,  19031,  19194,  19357,  19519,  19680,  19840,  20000,
    20159,  20317,  20474,  20631,  20787,  20942,  21096,  21249,  21402,
    21554,  21705,  21855,  22004,  22153,  22301,  22448,  22594,  22739,
    22883,  23027,  23169,  23311,  23452,  23592,  23731,  23869,  24006,
    24143,  24278,  24413,  24546,  24679,  24811,  24942,  25072,  25201,
    25329,  25456,  25582,  25707,  25831,  25954,  26077,  26198,  26318,
    26437,  26556,  26673,  26789,  26905,  27019,  27132,  27244,  27355,
    27466,  27575,  27683,  27790,  27896,  28001,  28105,  28208,  28309,
    28410,  28510,  28608,  28706,  28802,  28897,  28992,  29085,  29177,
    29268,  29358,  29446,  29534,  29621,  29706,  29790,  29873,  29955,
    30036,  30116,  30195,  30272,  30349,  30424,  30498,  30571,  30643,
    30713,  30783,  30851,  30918,  30984,  31049,  31113,  31175,  31236,
    31297,  31356,  31413,  31470,  31525,  31580,  31633,  31684,  31735,
    31785,  31833,  31880,  31926,  31970,  32014,  32056,  32097,  32137,
    32176,  32213,  32249,  32284,  32318,  32350,  32382,  32412,  32441,
    32468,  32495,  32520,  32544,  32567,  32588,  32609,  32628,  32646,
    32662,  32678,  32692,  32705,  32717,  32727,  32736,  32744,  32751,
    32757,  32761,  32764,  32766,  32767,  32766,  32764,  32761,  32757,
    32751,  32744,  32736,  32727,  32717,  32705,  32692,  32678,  32662,
    32646,  32628,  32609,  32588,  32567,  32544,  32520,  32495,  32468,
    32441,  32412,  32382,  32350,  32318,  32284,  32249,  32213,  32176,
    32137,  32097,  32056,  32014,  31970,  31926,  31880,  31833,  31785,
    31735,  31684,  31633,  31580,  31525,  31470,  31413,  31356,  31297,
    31236,  31175,  31113,  31049,  30984,  30918,  30851,  30783,  30713,
    30643,  30571,  30498,  30424,  30349,  30272,  30195,  30116,  30036,
    29955,  29873,  29790,  29706,  29621,  29534,  29446,  29358,  29268,
    29177,  29085,  28992,  28897,  28802,  28706,  28608,  28510,  28410,
    28309,  28208,  28105,  28001,  27896,  27790,  27683,  27575,  27466,
    27355,  27244,  27132,  27019,  26905,  26789,  26673,  26556,  26437,
    26318,  26198,  26077,  25954,  25831,  25707,  25582,  25456,  25329,
    25201,  25072,  24942,  24811,  24679,  24546,  24413,  24278,  24143,
    24006,  23869,  23731,  23592,  23452,  23311,  23169,  23027,  22883,
    22739,  22594,  22448,  22301,  22153,  22004,  21855,  21705,  21554,
    21402,  21249,  21096,  20942,  20787,  20631,  20474,  20317,  20159,
    20000,  19840,  19680,  19519,  19357,  19194,  19031,  18867,  18702,
    18537,  18371,  18204,  18036,  17868,  17699,  17530,  17360,  17189,
    17017,  16845,  16672,  16499,  16325,  16150,  15975,  15799,  15623,
    15446,  15268,  15090,  14911,  14732,  14552,  14372,  14191,  14009,
    13827,  13645,  13462,  13278,  13094,  12909,  12724,  12539,  12353,
    12166,  11980,  11792,  11604,  11416,  11227,  11038,  10849,  10659,
    10469,  10278,  10087,  9895,   9703,   9511,   9319,   9126,   8932,
    8739,   8545,   8351,   8156,   7961,   7766,   7571,   7375,   7179,
    6982,   6786,   6589,   6392,   6195,   5997,   5799,   5601,   5403,
    5205,   5006,   4807,   4608,   4409,   4210,   4011,   3811,   3611,
    3411,   3211,   3011,   2811,   2610,   2410,   2209,   2009,   1808,
    1607,   1406,   1206,   1005,   804,    603,    402,    201,    0,
    -201,   -402,   -603,   -804,   -1005,  -1206,  -1406,  -1607,  -1808,
    -2009,  -2209,  -2410,  -2610,  -2811,  -3011,  -3211,  -3411,  -3611,
    -3811,  -4011,  -4210,  -4409,  -4608,  -4807,  -5006,  -5205,  -5403,
    -5601,  -5799,  -5997,  -6195,  -6392,  -6589,  -6786,  -6982,  -7179,
    -7375,  -7571,  -7766,  -7961,  -8156,  -8351,  -8545,  -8739,  -8932,
    -9126,  -9319,  -9511,  -9703,  -9895,  -10087, -10278, -10469, -10659,
    -10849, -11038, -11227, -11416, -11604, -11792, -11980, -12166, -12353,
    -12539, -12724, -12909, -13094, -13278, -13462, -13645, -13827, -14009,
    -14191, -14372, -14552, -14732, -14911, -15090, -15268, -15446, -15623,
    -15799, -15975, -16150, -16325, -16499, -16672, -16845, -17017, -17189,
    -17360, -17530, -17699, -17868, -18036, -18204, -18371, -18537, -18702,
    -18867, -19031, -19194, -19357, -19519, -19680, -19840, -20000, -20159,
    -20317, -20474, -20631, -20787, -20942, -21096, -21249, -21402, -21554,
    -21705, -21855, -22004, -22153, -22301, -22448, -22594, -22739, -22883,
    -23027, -23169, -23311, -23452, -23592, -23731, -23869, -24006, -24143,
    -24278, -24413, -24546, -24679, -24811, -24942, -25072, -25201, -25329,
    -25456, -25582, -25707, -25831, -25954, -26077, -26198, -26318, -26437,
    -26556, -26673, -26789, -26905, -27019, -27132, -27244, -27355, -27466,
    -27575, -27683, -27790, -27896, -28001, -28105, -28208, -28309, -28410,
    -28510, -28608, -28706, -28802, -28897, -28992, -29085, -29177, -29268,
    -29358, -29446, -29534, -29621, -29706, -29790, -29873, -29955, -30036,
    -30116, -30195, -30272, -30349, -30424, -30498, -30571, -30643, -30713,
    -30783, -30851, -30918, -30984, -31049, -31113, -31175, -31236, -31297,
    -31356, -31413, -31470, -31525, -31580, -31633, -31684, -31735, -31785,
    -31833, -31880, -31926, -31970, -32014, -32056, -32097, -32137, -32176,
    -32213, -32249, -32284, -32318, -32350, -32382, -32412, -32441, -32468,
    -32495, -32520, -32544, -32567, -32588, -32609, -32628, -32646, -32662,
    -32678, -32692, -32705, -32717, -32727, -32736, -32744, -32751, -32757,
    -32761, -32764, -32766, -32767, -32766, -32764, -32761, -32757, -32751,
    -32744, -32736, -32727, -32717, -32705, -32692, -32678, -32662, -32646,
    -32628, -32609, -32588, -32567, -32544, -32520, -32495, -32468, -32441,
    -32412, -32382, -32350, -32318, -32284, -32249, -32213, -32176, -32137,
    -32097, -32056, -32014, -31970, -31926, -31880, -31833, -31785, -31735,
    -31684, -31633, -31580, -31525, -31470, -31413, -31356, -31297, -31236,
    -31175, -31113, -31049, -30984, -30918, -30851, -30783, -30713, -30643,
    -30571, -30498, -30424, -30349, -30272, -30195, -30116, -30036, -29955,
    -29873, -29790, -29706, -29621, -29534, -29446, -29358, -29268, -29177,
    -29085, -28992, -28897, -28802, -28706, -28608, -28510, -28410, -28309,
    -28208, -28105, -28001, -27896, -27790, -27683, -27575, -27466, -27355,
    -27244, -27132, -27019, -26905, -26789, -26673, -26556, -26437, -26318,
    -26198, -26077, -25954, -25831, -25707, -25582, -25456, -25329, -25201,
    -25072, -24942, -24811, -24679, -24546, -24413, -24278, -24143, -24006,
    -23869, -23731, -23592, -23452, -23311, -23169, -23027, -22883, -22739,
    -22594, -22448, -22301, -22153, -22004, -21855, -21705, -21554, -21402,
    -21249, -21096, -20942, -20787, -20631, -20474, -20317, -20159, -20000,
    -19840, -19680, -19519, -19357, -19194, -19031, -18867, -18702, -18537,
    -18371, -18204, -18036, -17868, -17699, -17530, -17360, -17189, -17017,
    -16845, -16672, -16499, -16325, -16150, -15975, -15799, -15623, -15446,
    -15268, -15090, -14911, -14732, -14552, -14372, -14191, -14009, -13827,
    -13645, -13462, -13278, -13094, -12909, -12724, -12539, -12353, -12166,
    -11980, -11792, -11604, -11416, -11227, -11038, -10849, -10659, -10469,
    -10278, -10087, -9895,  -9703,  -9511,  -9319,  -9126,  -8932,  -8739,
    -8545,  -8351,  -8156,  -7961,  -7766,  -7571,  -7375,  -7179,  -6982,
    -6786,  -6589,  -6392,  -6195,  -5997,  -5799,  -5601,  -5403,  -5205,
    -5006,  -4807,  -4608,  -4409,  -4210,  -4011,  -3811,  -3611,  -3411,
    -3211,  -3011,  -2811,  -2610,  -2410,  -2209,  -2009,  -1808,  -1607,
    -1406,  -1206,  -1005,  -804,   -603,   -402,   -201
};

#define CFFTSFT 14
#define CFFTRND 1
#define CFFTRND2 16384

#define CIFFTSFT 14
#define CIFFTRND 1

static const int16_t kBitReverseIndex7[112] = {
    1,   64,  2,   32,  3,   96,  4,   16,  5,   80,  6,   48,  7,   112,
    9,   72,  10,  40,  11,  104, 12,  24,  13,  88,  14,  56,  15,  120,
    17,  68,  18,  36,  19,  100, 21,  84,  22,  52,  23,  116, 25,  76,
    26,  44,  27,  108, 29,  92,  30,  60,  31,  124, 33,  66,  35,  98,
    37,  82,  38,  50,  39,  114, 41,  74,  43,  106, 45,  90,  46,  58,
    47,  122, 49,  70,  51,  102, 53,  86,  55,  118, 57,  78,  59,  110,
    61,  94,  63,  126, 67,  97,  69,  81,  71,  113, 75,  105, 77,  89,
    79,  121, 83,  101, 87,  117, 91,  109, 95,  125, 103, 115, 111, 123,
};

// リングバッファの読み出し要求を最大2つの連続領域に分割する。
// memcpy量を抑えながら外部バッファへ渡すための補助関数。
size_t GetBufferReadRegions(RingBuffer* buf,
                            size_t element_count,
                            void** data_ptr_1,
                            size_t* data_ptr_bytes_1,
                            void** data_ptr_2,
                            size_t* data_ptr_bytes_2) {
  const size_t readable = available_read(buf);
  const size_t to_read = readable < element_count ? readable : element_count;
  const size_t margin = buf->element_count - buf->read_pos;

  if (to_read > margin) {
    *data_ptr_1 = buf->data + buf->read_pos * buf->element_size;
    *data_ptr_bytes_1 = margin * buf->element_size;
    *data_ptr_2 = buf->data;
    *data_ptr_bytes_2 = (to_read - margin) * buf->element_size;
  } else {
    *data_ptr_1 = buf->data + buf->read_pos * buf->element_size;
    *data_ptr_bytes_1 = to_read * buf->element_size;
    *data_ptr_2 = NULL;
    *data_ptr_bytes_2 = 0;
  }

  return to_read;
}

// リングバッファの読み書き状態を初期化し、残留データをクリアする。
// 再利用前に既知の状態へ戻すためのエントリポイント。
void InitBuffer(RingBuffer* self) {
  if (!self) {
    return;
  }
  self->read_pos = 0;
  self->write_pos = 0;
  self->rw_wrap = SAME_WRAP;
  if (self->data && self->element_count && self->element_size) {
    memset(self->data, 0, self->element_count * self->element_size);
  }
}

// 指定されたメモリ領域をリングバッファとして設定し初期化する。
// 外部確保したバッファを循環バッファ運用に切り替えるためのAPI。
void InitBufferWith(RingBuffer* self,
                    void* backing,
                    size_t element_count,
                    size_t element_size) {
  if (!self || !backing || element_count == 0 || element_size == 0) {
    return;
  }
  self->data = static_cast<char*>(backing);
  self->element_count = element_count;
  self->element_size = element_size;
  InitBuffer(self);
}

// 現在取り出せる要素数を計算する。
// 読み出し前に不足を検知し安全な処理順を組むために使う。
size_t available_read(const RingBuffer* self) {
  if (!self) {
    return 0;
  }
  if (self->rw_wrap == SAME_WRAP) {
    return self->write_pos - self->read_pos;
  }
  return self->element_count - self->read_pos + self->write_pos;
}

// 現在書き込める空き要素数を返す。
// バッファ溢れを避けるための事前チェックに利用する。
size_t available_write(const RingBuffer* self) {
  if (!self) {
    return 0;
  }
  return self->element_count - available_read(self);
}

// 要求した要素数ぶんをリングバッファから読み出す。
// ストリーム処理が連続データを安全に取得できるよう抽象化する。
size_t ReadBuffer(RingBuffer* self,
                  void** data_ptr,
                  void* data,
                  size_t element_count) {
  if (!self || !data) {
    return 0;
  }

  void* buf_ptr_1 = NULL;
  void* buf_ptr_2 = NULL;
  size_t buf_bytes_1 = 0;
  size_t buf_bytes_2 = 0;
  const size_t read_count =
      GetBufferReadRegions(self, element_count, &buf_ptr_1, &buf_bytes_1,
                           &buf_ptr_2, &buf_bytes_2);
  if (buf_bytes_2 > 0) {
    memcpy(data, buf_ptr_1, buf_bytes_1);
    memcpy(static_cast<char*>(data) + buf_bytes_1, buf_ptr_2, buf_bytes_2);
    buf_ptr_1 = data;
  } else if (!data_ptr) {
    memcpy(data, buf_ptr_1, buf_bytes_1);
  }

  if (data_ptr) {
    *data_ptr = read_count == 0 ? NULL : buf_ptr_1;
  }

  MoveReadPtr(self, static_cast<int>(read_count));
  return read_count;
}

// 外部データをリングバッファへ書き込み、書き込めた要素数を返す。
// 入力ストリームを循環バッファへ蓄積する基本操作。
size_t WriteBuffer(RingBuffer* self,
                   const void* data,
                   size_t element_count) {
  if (!self || !data) {
    return 0;
  }

  const size_t free_elements = available_write(self);
  const size_t to_write = free_elements < element_count ? free_elements : element_count;
  size_t remaining = to_write;
  const size_t margin = self->element_count - self->write_pos;

  if (to_write > margin) {
    memcpy(self->data + self->write_pos * self->element_size,
           data, margin * self->element_size);
    self->write_pos = 0;
    remaining -= margin;
    self->rw_wrap = DIFF_WRAP;
  }

  memcpy(self->data + self->write_pos * self->element_size,
         static_cast<const char*>(data) + (to_write - remaining) * self->element_size,
         remaining * self->element_size);
  self->write_pos += remaining;

  return to_write;
}

// 読み出しポインタを相対移動させ、巻き戻り状態を更新する。
// 消費済み領域の解放や巻き戻しを一元的に扱うためのヘルパ。
int MoveReadPtr(RingBuffer* self, int element_count) {
  if (!self) {
    return 0;
  }

  const int free_elements = static_cast<int>(available_write(self));
  const int readable_elements = static_cast<int>(available_read(self));
  int read_pos = static_cast<int>(self->read_pos);

  if (element_count > readable_elements) {
    element_count = readable_elements;
  }
  if (element_count < -free_elements) {
    element_count = -free_elements;
  }

  read_pos += element_count;
  if (read_pos > static_cast<int>(self->element_count)) {
    read_pos -= static_cast<int>(self->element_count);
    self->rw_wrap = SAME_WRAP;
  }
  if (read_pos < 0) {
    read_pos += static_cast<int>(self->element_count);
    self->rw_wrap = DIFF_WRAP;
  }

  self->read_pos = static_cast<size_t>(read_pos);
  return element_count;
}

// 固定長のビット反転テーブルを用い、複素配列をビット反転順に並べ替える。
// FFT の前処理として、バタフライ演算を正しいペア順に整列させる。
void ComplexBitReverse(int16_t* __restrict complex_data, int stages) {
  (void)stages;
  int32_t* ptr = reinterpret_cast<int32_t*>(complex_data);
  for (int m = 0; m < 112; m += 2) {
    int32_t tmp = ptr[kBitReverseIndex7[m]];
    ptr[kBitReverseIndex7[m]] = ptr[kBitReverseIndex7[m + 1]];
    ptr[kBitReverseIndex7[m + 1]] = tmp;
  }
}

#define CFFTSFT 14
#define CFFTRND 1
#define CFFTRND2 16384

#define CIFFTSFT 14
#define CIFFTRND 1

// 32ビット整数の先頭に並ぶゼロビット数を数える。
// 正規化やスケール調整のシフト量決定に利用する基礎ルーチン。
int CountLeadingZeros32(uint32_t n) {
  if (n == 0) {
    return 32;
  }
  int count = 0;
  while ((n & 0x80000000u) == 0) {
    n <<= 1;
    ++count;
  }
  return count;
}

// 64ビット整数の先頭ゼロビット数を求める。
// 大きめの積や累積値を規格化する際の共通計算として使う。
int CountLeadingZeros64(uint64_t n) {
  if (n == 0) {
    return 64;
  }
  int count = 0;
  while ((n & 0x8000000000000000ULL) == 0) {
    n <<= 1;
    ++count;
  }
  return count;
}

// 32ビットの値を16ビット領域へ飽和クリップして格納する。
// 固定小数点演算で桁あふれを防ぐための基本変換。
int16_t SatW32ToW16(int32_t value32) {
  if (value32 > 32767) {
    return 32767;
  }
  if (value32 < -32768) {
    return -32768;
  }
  return (int16_t)value32;
}

// 32ビット整数を飽和付きで加算する。
// 累積計算でオーバーフローを避ける安全な加算ラッパー。
int32_t AddSatW32(int32_t a, int32_t b) {
  const int32_t sum = (int32_t)((uint32_t)a + (uint32_t)b);
  if ((a < 0) == (b < 0) && (a < 0) != (sum < 0)) {
    return sum < 0 ? INT32_MAX : INT32_MIN;
  }
  return sum;
}

// 32ビット整数の差分を飽和付きで求める。
// 減算時の符号反転による範囲逸脱を防ぐ。
int32_t SubSatW32(int32_t a, int32_t b) {
  const int32_t diff = (int32_t)((uint32_t)a - (uint32_t)b);
  if ((a < 0) != (b < 0) && (a < 0) != (diff < 0)) {
    return diff < 0 ? INT32_MAX : INT32_MIN;
  }
  return diff;
}

// 16ビット整数同士を飽和付きで加算する。
// PCMサンプルなど狭いダイナミックレンジの累積に使う。
int16_t AddSatW16(int16_t a, int16_t b) {
  return SatW32ToW16((int32_t)a + (int32_t)b);
}

// 16ビット整数の差を飽和付きで計算する。
// 固定小数点演算でのアンダーフロー対策として用いる。
int16_t SubSatW16(int16_t a, int16_t b) {
  return SatW32ToW16((int32_t)a - (int32_t)b);
}

// 32ビット整数を正規化するためのシフト量を算出する。
// 演算結果をオーバーフローさせずにスケーリングする準備に使う。
int16_t NormW32(int32_t a) {
  if (a == 0) {
    return 0;
  }
  int32_t tmp = a < 0 ? ~a : a;
  return (int16_t)(CountLeadingZeros32((uint32_t)tmp) - 1);
}

// 符号なし32ビット値を正規化するシフト量を返す。
// エネルギーなど非負値の規格化に用いる補助。
int16_t NormU32(uint32_t a) {
  return a == 0 ? 0 : (int16_t)CountLeadingZeros32(a);
}

// 16ビット整数の正規化に必要なビットシフト量を計算する。
// 小さなサンプルを扱う際に桁落ちを避けるための前処理。
int16_t NormW16(int16_t a) {
  if (a == 0) {
    return 0;
  }
  int32_t a32 = a;
  int32_t tmp = a < 0 ? ~a32 : a32;
  return (int16_t)(CountLeadingZeros32((uint32_t)tmp) - 17);
}

// 16ビットの内積を1項分計算して累積値へ加える。
// FIRや相関計算など逐次積和演算の基本プリミティブ。
int32_t MulAccumW16(int16_t a, int16_t b, int32_t c) {
  return a * b + c;
}

// 非ゼロ値を表現するのに必要なビット数を1始まりで返す。
// テーブル選択や指数表現の枝分かれ条件に使う。
int16_t GetSizeInBits(uint32_t n) {
  return (int16_t)(32 - CountLeadingZeros32(n));
}

// 16ビット配列内の最大絶対値を線形探索で求める。
// スケーリング判断やレベル検出の前処理に利用する。
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length) {
  int maximum = 0;
  for (size_t i = 0; i < length; ++i) {
    int absolute = abs((int)vector[i]);
    if (absolute > maximum) {
      maximum = absolute;
    }
  }
  if (maximum > 32767) {
    maximum = 32767;
  }
  return (int16_t)maximum;
}

int16_t ExtractFractionPart(uint32_t a, int zeros) {
  return static_cast<int16_t>(((a << zeros) & 0x7FFFFFFF) >> 23);
}

int16_t LogOfEnergyInQ8(uint32_t energy, int q_domain) {
  const int16_t kLogLowValue = (1 << kRealFftOrder) << 7;
  int16_t log_energy_q8 = kLogLowValue;
  if (energy > 0) {
    int zeros = NormU32(energy);
    int16_t frac = ExtractFractionPart(energy, zeros);
    log_energy_q8 += ((31 - zeros) << 8) + frac - (q_domain << 8);
  }
  return log_energy_q8;
}

// 32ビット符号なし値を16ビットで割り、安全に商を返す。
// ゼロ除算をガードしつつ比率計算を行うためのラッパー。
uint32_t DivU32U16(uint32_t num, uint16_t den) {
  return den == 0 ? UINT32_MAX : (uint32_t)(num / den);
}

// 32ビット符号付き値を16ビットで割り、ゼロ除算時は最大値を返す。
// ゲインや係数を整数演算で求める際の安全な除算。
int32_t DivW32W16(int32_t num, int16_t den) {
  return den == 0 ? INT32_MAX : (int32_t)(num / den);
}

// 32ビット整数の平方根を繰り返し法で近似し、床値を返す。
// 浮動小数点を使わずに振幅やエネルギーのルートを評価するために利用。
int32_t SqrtFloor(int32_t value) {
  if (value <= 0) {
    return 0;
  }
  int32_t root = 0;
  for (int i = 15; i >= 0; --i) {
    int32_t try1 = root + (1 << i);
    int32_t shifted = try1 << i;
    if (value >= shifted) {
      value -= shifted;
      root |= 2 << i;
    }
  }
  return root >> 1;
}

// 固定長128ポイントまでの複素FFTを実装したルーチン。
// 周波数領域での解析や畳み込み前処理として利用する。
int ComplexFFT(int16_t frfi[], int stages, int mode) {
  (void)mode;
  const int n = 1 << stages;
  if (n > 1024) {
    return -1;
  }

  int l = 1;
  int k = 10 - 1;

  while (l < n) {
    int istep = l << 1;
    for (int m = 0; m < l; ++m) {
      int j = m << k;
      int16_t wr = kSinTable1024[j + 256];
      int16_t wi = -kSinTable1024[j];

      for (int i = m; i < n; i += istep) {
        j = i + l;

        int32_t tr32 = wr * frfi[2 * j] - wi * frfi[2 * j + 1] + CFFTRND;
        int32_t ti32 = wr * frfi[2 * j + 1] + wi * frfi[2 * j] + CFFTRND;

        tr32 >>= 15 - CFFTSFT;
        ti32 >>= 15 - CFFTSFT;

        int32_t qr32 = ((int32_t)frfi[2 * i]) * (1 << CFFTSFT);
        int32_t qi32 = ((int32_t)frfi[2 * i + 1]) * (1 << CFFTSFT);

        frfi[2 * j] = (int16_t)((qr32 - tr32 + CFFTRND2) >> (1 + CFFTSFT));
        frfi[2 * j + 1] = (int16_t)((qi32 - ti32 + CFFTRND2) >> (1 + CFFTSFT));
        frfi[2 * i] = (int16_t)((qr32 + tr32 + CFFTRND2) >> (1 + CFFTSFT));
        frfi[2 * i + 1] = (int16_t)((qi32 + ti32 + CFFTRND2) >> (1 + CFFTSFT));
      }
    }
    --k;
    l = istep;
  }
  return 0;
}

// 複素周波数データを時間領域に戻すIFFTを行い、必要なスケーリング量を返す。
// 周波数領域処理後の再構成やオーバーラップアドに欠かせないコア処理。
int ComplexIFFT(int16_t frfi[], int stages, int mode) {
  (void)mode;
  const size_t n = ((size_t)1) << stages;
  if (n > 1024) {
    return -1;
  }

  int scale = 0;
  size_t l = 1;
  int k = 10 - 1;

  while (l < n) {
    int shift = 0;
    int32_t round2 = 8192;

    int32_t tmp32 = MaxAbsValueW16(frfi, 2 * n);
    if (tmp32 > 13573) {
      ++shift;
      ++scale;
      round2 <<= 1;
    }
    if (tmp32 > 27146) {
      ++shift;
      ++scale;
      round2 <<= 1;
    }

    size_t istep = l << 1;
    for (size_t m = 0; m < l; ++m) {
      size_t j = m << k;
      int16_t wr = kSinTable1024[j + 256];
      int16_t wi = kSinTable1024[j];

      for (size_t i = m; i < n; i += istep) {
        j = i + l;

        int32_t tr32 = wr * frfi[2 * j] - wi * frfi[2 * j + 1] + CIFFTRND;
        int32_t ti32 = wr * frfi[2 * j + 1] + wi * frfi[2 * j] + CIFFTRND;

        tr32 >>= 15 - CIFFTSFT;
        ti32 >>= 15 - CIFFTSFT;

        int32_t qr32 = ((int32_t)frfi[2 * i]) * (1 << CIFFTSFT);
        int32_t qi32 = ((int32_t)frfi[2 * i + 1]) * (1 << CIFFTSFT);

        frfi[2 * j] = (int16_t)((qr32 - tr32 + round2) >> (shift + CIFFTSFT));
        frfi[2 * j + 1] = (int16_t)((qi32 - ti32 + round2) >> (shift + CIFFTSFT));
        frfi[2 * i] = (int16_t)((qr32 + tr32 + round2) >> (shift + CIFFTSFT));
        frfi[2 * i + 1] = (int16_t)((qi32 + ti32 + round2) >> (shift + CIFFTSFT));
      }
    }
    --k;
    l = istep;
  }
  return scale;
}

// 実数波形を複素スペクトルへ変換し、DC〜Nyquist成分を取得する。
// 実信号ベースのAECM処理が周波数領域に移行する前段として利用。
int RealForwardFFT(const int16_t* real_data_in,
                   int16_t* complex_data_out) {
  const int n = 1 << kRealFftOrder;
  int16_t complex_buffer[2 << kRealFftOrder];

  for (int i = 0, j = 0; i < n; ++i, j += 2) {
    complex_buffer[j] = real_data_in[i];
    complex_buffer[j + 1] = 0;
  }

  ComplexBitReverse(complex_buffer, kRealFftOrder);
  int result = ComplexFFT(complex_buffer, kRealFftOrder, 1);

  memcpy(complex_data_out, complex_buffer, sizeof(int16_t) * (n + 2));
  return result;
}

// 実数スペクトルを逆変換し、時間領域のサンプル列へ復元する。
// 周波数領域処理後のブロックを時間波形に戻して合成する工程で使用。
int RealInverseFFT(const int16_t* complex_data_in,
                   int16_t* real_data_out) {
  const int n = 1 << kRealFftOrder;
  int16_t complex_buffer[2 << kRealFftOrder];

  memcpy(complex_buffer, complex_data_in, sizeof(int16_t) * (n + 2));
  for (int i = n + 2; i < 2 * n; i += 2) {
    complex_buffer[i] = complex_data_in[2 * n - i];
    complex_buffer[i + 1] = -complex_data_in[2 * n - i + 1];
  }

  ComplexBitReverse(complex_buffer, kRealFftOrder);
  int result = ComplexIFFT(complex_buffer, kRealFftOrder, 1);

  for (int i = 0, j = 0; i < n; ++i, j += 2) {
    real_data_out[i] = complex_buffer[j];
  }

  return result;
}

void InverseFFTAndWindow(int16_t* fft,
                         ComplexInt16* efw,
                         int part_len,
                         int part_len2,
                         const int16_t* sqrt_hanning,
                         int16_t* current_block,
                         int16_t* overlap_block) {
  int16_t* ifft_out = reinterpret_cast<int16_t*>(efw);

  for (int i = 1, j = 2; i < part_len; ++i, j += 2) {
    fft[j] = efw[i].real;
    fft[j + 1] = -efw[i].imag;
  }
  fft[0] = efw[0].real;
  fft[1] = -efw[0].imag;

  fft[part_len2] = efw[part_len].real;
  fft[part_len2 + 1] = -efw[part_len].imag;

  int outCFFT = RealInverseFFT(fft, ifft_out);
  for (int i = 0; i < part_len; ++i) {
    ifft_out[i] = static_cast<int16_t>(
        MUL_16_16_RSFT_WITH_ROUND(ifft_out[i], sqrt_hanning[i], 14));
    int32_t tmp32 = SHIFT_W32(static_cast<int32_t>(ifft_out[i]), outCFFT);
    current_block[i] = static_cast<int16_t>(SAT(WORD16_MAX, tmp32, WORD16_MIN));

    tmp32 = (ifft_out[part_len + i] * sqrt_hanning[part_len - i]) >> 14;
    tmp32 = SHIFT_W32(tmp32, outCFFT);
    overlap_block[i] = static_cast<int16_t>(SAT(WORD16_MAX, tmp32, WORD16_MIN));
  }
}

void WindowAndFFT(int16_t* fft,
                  const int16_t* time_signal,
                  ComplexInt16* freq_signal,
                  int part_len,
                  const int16_t* sqrt_hanning) {
  for (int i = 0; i < part_len; ++i) {
    int16_t scaled = time_signal[i];
    fft[i] = static_cast<int16_t>((scaled * sqrt_hanning[i]) >> 14);
    scaled = time_signal[i + part_len];
    fft[part_len + i] = static_cast<int16_t>((scaled * sqrt_hanning[part_len - i]) >> 14);
  }

  RealForwardFFT(fft, reinterpret_cast<int16_t*>(freq_signal));
  for (int i = 0; i < part_len; ++i) {
    freq_signal[i].imag = -freq_signal[i].imag;
  }
}
