#include "signal_processing_library.h"

/* Tables for data buffer indexes that are bit reversed and thus need to be
 * swapped. Note that, index_7[{0, 2, 4, ...}] are for the left side of the swap
 * operations, while index_7[{1, 3, 5, ...}] are for the right side of the
 * operation. Same for index_8.
 */

/* Indexes for the case of stages == 7. */
static const int16_t index_7[112] = {
  1, 64, 2, 32, 3, 96, 4, 16, 5, 80, 6, 48, 7, 112, 9, 72, 10, 40, 11, 104,
  12, 24, 13, 88, 14, 56, 15, 120, 17, 68, 18, 36, 19, 100, 21, 84, 22, 52,
  23, 116, 25, 76, 26, 44, 27, 108, 29, 92, 30, 60, 31, 124, 33, 66, 35, 98,
  37, 82, 38, 50, 39, 114, 41, 74, 43, 106, 45, 90, 46, 58, 47, 122, 49, 70,
  51, 102, 53, 86, 55, 118, 57, 78, 59, 110, 61, 94, 63, 126, 67, 97, 69,
  81, 71, 113, 75, 105, 77, 89, 79, 121, 83, 101, 87, 117, 91, 109, 95, 125,
  103, 115, 111, 123
};

/*
 * 本プロジェクトは order=7（PART_LEN=64）固定。
 * 汎用/他オーダ対応を捨て、stage==7のテーブルのみでビットリバースを行う。
 */
void ComplexBitReverse(int16_t* __restrict complex_data, int stages) {
  (void)stages;  // 16k/64pt固定のため未使用

  const int length = 112;
  const int16_t* index = index_7;

  /* Decimation in time. Swap the elements with bit-reversed indexes. */
  for (int m = 0; m < length; m += 2) {
    /* 16-bitの実部/虚部をまとめて32-bitとして交換する */
    int32_t* complex_data_ptr = (int32_t*)complex_data;
    int32_t temp = complex_data_ptr[index[m]];  /* Real and imaginary */
    complex_data_ptr[index[m]] = complex_data_ptr[index[m + 1]];
    complex_data_ptr[index[m + 1]] = temp;
  }
}
