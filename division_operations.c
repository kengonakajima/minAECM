/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


/*
 * This file contains implementations of the divisions
 * DivU32U16()
 * DivW32W16()
 * （教育用最小構成では未使用のため削除: DivW32W16ResW16, DivResultInQ31, DivW32HiLow）
 *
 * The description header can be found in signal_processing_library.h
 *
 */

#include "signal_processing_library.h"

uint32_t DivU32U16(uint32_t num, uint16_t den)
{
    // Guard against division with 0
    if (den != 0)
    {
        return (uint32_t)(num / den);
    } else
    {
        return (uint32_t)0xFFFFFFFF;
    }
}

int32_t DivW32W16(int32_t num, int16_t den)
{
    // Guard against division with 0
    if (den != 0)
    {
        return (int32_t)(num / den);
    } else
    {
        return (int32_t)0x7FFFFFFF;
    }
}

// 以降の除算ユーティリティは未使用につき削除
