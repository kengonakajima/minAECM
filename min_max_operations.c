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
 * MaxAbsValueW16C()
 * MaxAbsValueW32C()
 * MaxValueW16C()
 * MaxValueW32C()
 * MinValueW16C()
 * MinValueW32C()
 * MaxAbsIndexW16()
 * MaxIndexW16()
 * MaxIndexW32()
 * MinIndexW16()
 * MinIndexW32()
 *
 */

#include <stdlib.h>

#include "signal_processing_library.h"

// TODO(bjorn/kma): Consolidate function pairs (e.g. combine
//   MaxAbsValueW16C and MaxAbsIndexW16 into a single one.)
// TODO(kma): Move the next six functions into min_max_operations_c.c.

// この翻訳単位では、公開 API 以外の補助関数は未使用のため削除。

// Maximum absolute value of word16 vector. C version for generic platforms.
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length) {
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
  if (maximum > WORD16_MAX) {
    maximum = WORD16_MAX;
  }

  return (int16_t)maximum;
}

// Maximum absolute value of word32 vector. C version for generic platforms.
int32_t MaxAbsValueW32C(const int32_t* vector, size_t length) {
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

  maximum = MIN(maximum, WORD32_MAX);

  return (int32_t)maximum;
}

// Maximum value of word16 vector. C version for generic platforms.
int16_t MaxValueW16C(const int16_t* vector, size_t length) {
  int16_t maximum = WORD16_MIN;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] > maximum)
      maximum = vector[i];
  }
  return maximum;
}

// Maximum value of word32 vector. C version for generic platforms.
int32_t MaxValueW32C(const int32_t* vector, size_t length) {
  int32_t maximum = WORD32_MIN;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] > maximum)
      maximum = vector[i];
  }
  return maximum;
}

// Minimum value of word16 vector. C version for generic platforms.
int16_t MinValueW16C(const int16_t* vector, size_t length) {
  int16_t minimum = WORD16_MAX;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] < minimum)
      minimum = vector[i];
  }
  return minimum;
}

// Minimum value of word32 vector. C version for generic platforms.
int32_t MinValueW32C(const int32_t* vector, size_t length) {
  int32_t minimum = WORD32_MAX;
  size_t i = 0;

  /* length > 0 を期待 */

  for (i = 0; i < length; i++) {
    if (vector[i] < minimum)
      minimum = vector[i];
  }
  return minimum;
}

// 未使用の補助ルーチン（MaxAbsIndex/MaxIndex/MinIndex など）は削除済み。
