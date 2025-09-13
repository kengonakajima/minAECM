#!/usr/bin/env python3
"""
WebRTC 系の接頭語を識別子先頭から取り除く一括置換ツール。

対象:
  - 先頭に現れる `WEBRTC_` / `webrtc_` / `WebRtc`

注意:
  - 文字列/コメント内も置換されます（サイズ削減が目的のため）。
  - バイナリやオブジェクト、アーカイブは対象外。
  - 競合（同名化）などはビルド時に検出してください。

使い方:
  ドライラン:  python tools/strip_webrtc_prefixes.py --dry-run
  反映する:    python tools/strip_webrtc_prefixes.py --write
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path


# 対象とするファイル拡張子（C/C++ メイン）
CODE_EXTS = {
    ".c", ".cc", ".cpp", ".cxx",
    ".h", ".hh", ".hpp", ".hxx",
}

# 除外するディレクトリ（生成物など）
EXCLUDE_DIRS = {
    ".git",
    "build",
    "out",
    "bin",
    "obj",
    "dist",
    "node_modules",
    "__pycache__",
    ".DS_Store",
    "*.dSYM",
}


def should_skip_dir(name: str) -> bool:
    # ワイルドカード簡易対応（*.dSYM）
    if name.endswith(".dSYM"):
        return True
    return name in EXCLUDE_DIRS


def iter_code_files(root: Path):
    for dirpath, dirnames, filenames in os.walk(root):
        # 除外ディレクトリをスキップ
        dirnames[:] = [d for d in dirnames if not should_skip_dir(d)]
        for f in filenames:
            p = Path(dirpath) / f
            if p.suffix in CODE_EXTS:
                yield p


def compile_patterns():
    # 先頭が識別子文字でない位置で始まり、直後が識別子継続可能な場合だけマッチ
    # (?<![A-Za-z0-9_]) の直後に対象プレフィックスが来た時に取り除く
    id_prev = r"(?<![A-Za-z0-9_])"
    id_next = r"(?=[A-Za-z0-9_])"
    patterns = [
        # まず WebRtc_ を優先的に除去（例: WebRtc_CreateFoo -> CreateFoo）
        (re.compile(id_prev + r"WebRtc_" + id_next), ""),
        # 次に他のバリエーション
        (re.compile(id_prev + r"WEBRTC_" + id_next), ""),
        (re.compile(id_prev + r"webrtc_" + id_next), ""),
        (re.compile(id_prev + r"WebRtc" + id_next), ""),
        # 最後に、トークン先頭に残った単一の '_' を落とす（__ は対象外）
        (re.compile(id_prev + r"_(?=[A-Za-z])"), ""),
    ]
    return patterns


def transform_content(text: str, patterns):
    total = 0
    for pat, repl in patterns:
        text, n = pat.subn(repl, text)
        total += n
    return text, total


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="探索ルート")
    parser.add_argument("--dry-run", action="store_true", help="変更件数のみ出力")
    parser.add_argument("--write", action="store_true", help="変更を書き戻す")
    args = parser.parse_args(argv)

    if args.dry_run and args.write:
        print("--dry-run と --write は同時指定できません", file=sys.stderr)
        return 2
    if not (args.dry_run or args.write):
        # デフォルトはドライラン
        args.dry_run = True

    patterns = compile_patterns()

    total_files = 0
    total_repl = 0
    changed_files = 0

    for p in iter_code_files(args.root):
        try:
            data = p.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            # バイナリ/非UTF-8はスキップ
            continue
        new_data, n = transform_content(data, patterns)
        total_files += 1
        if n:
            changed_files += 1
            total_repl += n
            if args.write:
                p.write_text(new_data, encoding="utf-8")

    mode = "DRY-RUN" if args.dry_run else "WRITE"
    print(f"[{mode}] 対象ファイル: {total_files} / 変更あり: {changed_files} / 置換件数: {total_repl}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
