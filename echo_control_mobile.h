/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_AECM_ECHO_CONTROL_MOBILE_H_
#define MODULES_AUDIO_PROCESSING_AECM_ECHO_CONTROL_MOBILE_H_

#include <stddef.h>
#include <stdint.h>

 

enum { AecmFalse = 0, AecmTrue };

// Errors
#define AECM_UNSPECIFIED_ERROR 12000
#define AECM_UNSUPPORTED_FUNCTION_ERROR 12001
#define AECM_UNINITIALIZED_ERROR 12002
#define AECM_NULL_POINTER_ERROR 12003
#define AECM_BAD_PARAMETER_ERROR 12004

// Warnings
#define AECM_BAD_PARAMETER_WARNING 12100

typedef struct {
  int16_t echoMode;  // 3 のみサポート
} AecmConfig;

/*
 * 単一インスタンス（シングルトン）前提の極簡API。
 * 16 kHz/モノラル固定・ブロック長=64（FRAME_LEN）固定。
 * 内部ではアプリ側ラッパの状態 `am` と、AECM コアの状態 `g_aecm` を
 * グローバルに保持します。Create/Free は不要で、ポインタ引数も不要です。
 */

/* 初期化（再初期化可）。0で成功。*/
int32_t Init();

/* Farend（参照）64サンプルを供給。0で成功。*/
int32_t BufferFarend(const int16_t* farend);

/* Nearend 64サンプルを処理して out へ。0で成功。
 * デバイス遅延は本実装では固定(50ms)として内部使用。*/
int32_t Process(const int16_t* nearend,
                     int16_t* out);

/* 動作モードの設定。0で成功。*/
int32_t SetConfig(AecmConfig config);

 

#endif  // MODULES_AUDIO_PROCESSING_AECM_ECHO_CONTROL_MOBILE_H_
