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
 * This file contains the implementation of functions used in minAECM:
 *   MaxAbsValueW16C()
 *
 */

#include <stdlib.h>

#include "signal_processing_library.h"

// Maximum absolute value of word16 vector. C version for generic platforms.
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length) {
  int maximum = 0;

  /* length > 0 を期待 */

  for (size_t i = 0; i < length; i++) {
    int absolute = abs((int)vector[i]);

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
