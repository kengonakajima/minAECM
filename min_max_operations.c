/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

/*
 * This file contains the implementation of functions
 * Spl_MaxAbsValueW16C()
 * Spl_MaxAbsValueW32C()
 * Spl_MaxValueW16C()
 * Spl_MaxValueW32C()
 * Spl_MinValueW16C()
 * Spl_MinValueW32C()
 * Spl_MaxAbsIndexW16()
 * Spl_MaxIndexW16()
 * Spl_MaxIndexW32()
 * Spl_MinIndexW16()
 * Spl_MinIndexW32()
 *
 */

#include <stdlib.h>

#include "signal_processing_library.h"

// TODO(bjorn/kma): Consolidate function pairs (e.g. combine
//   Spl_MaxAbsValueW16C and Spl_MaxAbsIndexW16 into a single one.)
// TODO(kma): Move the next six functions into min_max_operations_c.c.

// この翻訳単位では、公開 API 以外の補助関数は未使用のため削除。

// Maximum absolute value of word16 vector. C version for generic platforms.
int16_t Spl_MaxAbsValueW16C(const int16_t* vector, size_t length) {
  size_t i = 0;
  int absolute = 0, maximum = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    absolute = abs((int)vector[i]);

    if (absolute > maximum) {
      maximum = absolute;
    }
  }

  // Guard the case for abs(-32768).
  if (maximum > SPL_WORD16_MAX) {
    maximum = SPL_WORD16_MAX;
  }

  return (int16_t)maximum;
}

// Maximum absolute value of word32 vector. C version for generic platforms.
int32_t Spl_MaxAbsValueW32C(const int32_t* vector, size_t length) {
  // Use uint32_t for the local variables, to accommodate the return value
  // of abs(0x80000000), which is 0x80000000.

  uint32_t absolute = 0, maximum = 0;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    absolute = abs((int)vector[i]);
    if (absolute > maximum) {
      maximum = absolute;
    }
  }

  maximum = SPL_MIN(maximum, SPL_WORD32_MAX);

  return (int32_t)maximum;
}

// Maximum value of word16 vector. C version for generic platforms.
int16_t Spl_MaxValueW16C(const int16_t* vector, size_t length) {
  int16_t maximum = SPL_WORD16_MIN;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] > maximum)
      maximum = vector[i];
  }
  return maximum;
}

// Maximum value of word32 vector. C version for generic platforms.
int32_t Spl_MaxValueW32C(const int32_t* vector, size_t length) {
  int32_t maximum = SPL_WORD32_MIN;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] > maximum)
      maximum = vector[i];
  }
  return maximum;
}

// Minimum value of word16 vector. C version for generic platforms.
int16_t Spl_MinValueW16C(const int16_t* vector, size_t length) {
  int16_t minimum = SPL_WORD16_MAX;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] < minimum)
      minimum = vector[i];
  }
  return minimum;
}

// Minimum value of word32 vector. C version for generic platforms.
int32_t Spl_MinValueW32C(const int32_t* vector, size_t length) {
  int32_t minimum = SPL_WORD32_MAX;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] < minimum)
      minimum = vector[i];
  }
  return minimum;
}

// 未使用の補助ルーチン（MaxAbsIndex/MaxIndex/MinIndex など）は削除済み。
