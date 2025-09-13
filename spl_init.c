/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// Some code came from common/rtcd.c in the WebM project.

#include "signal_processing_library.h"

// CPUタイプ検出と最適化分岐は削除。関数ポインタによるランタイム選択も廃止。
