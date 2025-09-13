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

// CPUタイプ検出と最適化分岐は削除。常に汎用C実装を指す。
const MaxAbsValueW16 WebRtcSpl_MaxAbsValueW16 = WebRtcSpl_MaxAbsValueW16C;
const MaxAbsValueW32 WebRtcSpl_MaxAbsValueW32 = WebRtcSpl_MaxAbsValueW32C;
const MaxValueW16 WebRtcSpl_MaxValueW16 = WebRtcSpl_MaxValueW16C;
const MaxValueW32 WebRtcSpl_MaxValueW32 = WebRtcSpl_MaxValueW32C;
const MinValueW16 WebRtcSpl_MinValueW16 = WebRtcSpl_MinValueW16C;
const MinValueW32 WebRtcSpl_MinValueW32 = WebRtcSpl_MinValueW32C;
const CrossCorrelation WebRtcSpl_CrossCorrelation = WebRtcSpl_CrossCorrelationC;
const DownsampleFast WebRtcSpl_DownsampleFast = WebRtcSpl_DownsampleFastC;
const ScaleAndAddVectorsWithRound WebRtcSpl_ScaleAndAddVectorsWithRound =
    WebRtcSpl_ScaleAndAddVectorsWithRoundC;
