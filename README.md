# minAECM

## LICENSE

このレポジトリのコードは、WebRTCのソースに含まれるaecmモジュールのソースコードを、
説明のために改変したものです。
説明のために、文字数を減らすために、ライセンスのコメントを削減しています。

もとのソースコードの各ファイルの冒頭部分には、以下の著作権表記が添付されています。


/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


## コンパイル
- Apple Silicon のmac
- macOS Sequoia 以降
- Xcode 26.0のコマンドラインツール

以上をインストールした状態で、
レポジトリトップで make
すればビルドできます。

以下のようにしてエコーバックサンプルを起動できます。

```
./echoback --input-delay-ms 50   # 人工的に50msの遅延を設定してエコーバック
./echoback --passthrough   # AECをオフにしてパススルーする。ハウリングします
```

