/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

// 任意データを保持するリングバッファ。スレッドセーフではありません。
// 断りがない限り、関数は成功時 0・失敗時 -1 を返します。

#include <stddef.h>  // size_t

enum Wrap { SAME_WRAP, DIFF_WRAP };

typedef struct RingBuffer {
  size_t read_pos;
  size_t write_pos;
  size_t element_count;
  size_t element_size;
  enum Wrap rw_wrap;
  char* data;
} RingBuffer;

// 初期化（固定配列バッキング利用）。
void InitBuffer(RingBuffer* handle);

// 事前確保した領域で初期化（動的確保なし）
void InitBufferWith(RingBuffer* handle,
                    void* backing,
                    size_t element_count,
                    size_t element_size);

// バッファからデータを読み出し、読み取った要素数を返します。
// `data_ptr` には読み出したデータが存在するアドレスが設定されます。
// 読み出せるデータが無い場合は `data_ptr` に `NULL` を設定します。
// ラップせず読み出せる場合は `data_ptr` がバッファ内の位置を指します。
// それ以外ではデータを `data` にコピーし（メモリ確保は呼び出し側）、
// `data_ptr` が `data` のアドレスを指します。`data_ptr` の有効期間は
// 次に WriteBuffer() を呼び出すまでです。
//
// 常に `data` へコピーさせたいときは `data_ptr` に NULL を渡します。
//
// 戻り値は読み出した要素数です。
size_t ReadBuffer(RingBuffer* handle,
                         void** data_ptr,
                         void* data,
                         size_t element_count);

// `data` をバッファへ書き込み、書き込んだ要素数を返します。
size_t WriteBuffer(RingBuffer* handle,
                          const void* data,
                          size_t element_count);

// 読み取り位置を移動し、移動した要素数を返します。
// `element_count` が正なら書き込み位置へ近づけ（バッファをフラッシュ）、
// 負なら書き込み位置から遠ざけます（バッファを詰める）。
// 戻り値は移動した要素数です。
int MoveReadPtr(RingBuffer* handle, int element_count);

// 読み出し可能な要素数を返します。
size_t available_read(const RingBuffer* handle);

// 書き込み可能な要素数を返します。
size_t available_write(const RingBuffer* handle);
