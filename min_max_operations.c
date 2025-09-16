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
 * このファイルには minAECM で利用する関数の実装が含まれています:
 *   MaxAbsValueW16C()
 *
 */

#include <stdlib.h>

#include "signal_processing_library.h"

// word16 ベクトルの絶対値最大を求める（汎用プラットフォーム向け C 実装）。
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length) {
  int maximum = 0;

  /* length > 0 を期待 */

  for (size_t i = 0; i < length; i++) {
    int absolute = abs((int)vector[i]);

    if (absolute > maximum) {
      maximum = absolute;
    }
  }

  // abs(-32768) のケースを保護。
  if (maximum > WORD16_MAX) {
    maximum = WORD16_MAX;
  }

  return (int16_t)maximum;
}
