# https://gist.github.com/azk-mochi/a36a53c9eb9a63bb5b70cfe4e288b560

PLATFORM=macosx
ARCH=arch -arm64

SDKROOT=$(shell xcrun --sdk $(PLATFORM) --show-sdk-path)
CXX=$(ARCH) $(shell xcrun --sdk $(PLATFORM) --find clang++)
CC=$(ARCH) $(shell xcrun --sdk $(PLATFORM) --find clang)
AR=$(shell xcrun --sdk $(PLATFORM) --find ar) 
RANLIB=$(shell xcrun --sdk $(PLATFORM) --find ranlib) 



INCFLAGS=-I. -isysroot $(SDKROOT)
PFFLAGS=-DWEBRTC_POSIX
# サードパーティコードのビルド安定化のため、unused関連のWerrorは外す
WARN_CXX=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-private-field -Wunused-variable -Wno-unused-parameter
WARN_C=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-variable -Wno-unused-parameter -Wmissing-prototypes
CPPFLAGS=$(INCFLAGS) $(PFFLAGS)
CXXFLAGS=-std=c++20 -g -O3 -mmacosx-version-min=10.13 \
  -ffunction-sections -fdata-sections $(WARN_CXX)
CFLAGS=-g -O3 -mmacosx-version-min=10.13 \
  -ffunction-sections -fdata-sections $(WARN_C)
LDFLAGS=-Wl,-dead_strip

# Emscripten (WASM) build configuration
EMCC=emcc
EMCPPFLAGS=-I.
EMCXXFLAGS=-O3 -std=c++17 -s MODULARIZE=1 -s ENVIRONMENT=node -s ALLOW_MEMORY_GROWTH=1 -s ASSERTIONS=0 \
  -s EXPORT_ES6=0 -s NO_EXIT_RUNTIME=1
EMLDFLAGS=-s EXPORTED_RUNTIME_METHODS='["ccall","cwrap"]' \
  -s EXPORTED_FUNCTIONS='["_malloc","_free","_aecm_create","_aecm_destroy","_aecm_reset","_aecm_process","_aecm_set_bypass_supmask","_aecm_set_bypass_nlp","_aecm_get_last_delay_blocks"]'
WASM_SRCS=wasm/aecm_wasm.cc aecm.cc delay_estimator.cc util.cc
WASM_OUT=dist/aecm_wasm.js

# C 実装も C++ としてビルドし、リンク指定子を単純化
%.o: %.c
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -x c++ -c $< -o $@


all: libaecm.a echoback cancel_file
	rm -f *.tmp


# ========================
# AECM ライブラリの構築
# ========================

# AECM に必要な最小ソース群（MIPS/NEON/テスト類は除外）
AECM_CC_SRCS= \
  aecm.cc \
  delay_estimator.cc \
  util.cc

# AECM の動作に必要な最小限の SPL/C 実装のみをビルド
# （未使用のLPC/フィルタ/ダウンサンプル/ユーティリティ群は除外）
AECM_C_SRCS=
 

AECM_OBJS=$(AECM_CC_SRCS:.cc=.o) $(AECM_C_SRCS:.c=.o)

libaecm.a : $(AECM_OBJS)
	$(AR) cr libaecm.a $(AECM_OBJS)
	$(RANLIB) libaecm.a


PORTAUDIO_HOME?=$(HOME)/portaudio
PA_FALLBACK_INCDIR1=/opt/homebrew/opt/portaudio/include
PA_FALLBACK_INCDIR2=/usr/local/include
PA_FALLBACK_LIBDIR1=/opt/homebrew/opt/portaudio/lib
PA_FALLBACK_LIBDIR2=/usr/local/lib

# Echoback: AECM を用いたローカル・エコーバック
echoback: echoback.cc libaecm.a libportaudio.a
	$(CXX) -o echoback $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) \
		-I$(PORTAUDIO_HOME)/include -I$(PA_FALLBACK_INCDIR1) -I$(PA_FALLBACK_INCDIR2) -I$(PORTAUDIO_HOME) \
		echoback.cc libaecm.a \
		-L$(PORTAUDIO_HOME) -L$(PORTAUDIO_HOME)/lib -L$(PA_FALLBACK_LIBDIR1) -L$(PA_FALLBACK_LIBDIR2) \
		-Wl,-rpath,$(PORTAUDIO_HOME) -Wl,-rpath,$(PORTAUDIO_HOME)/lib -Wl,-rpath,$(PA_FALLBACK_LIBDIR1) -Wl,-rpath,$(PA_FALLBACK_LIBDIR2) \
		./libportaudio.a -framework CoreAudio -framework AudioToolbox -framework AudioUnit -framework CoreServices 

# Offline comparator (no PortAudio)
cancel_file: cancel_file.cc libaecm.a
	$(CXX) -o cancel_file $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) \
		cancel_file.cc libaecm.a

wasm: $(WASM_OUT)

$(WASM_OUT): $(WASM_SRCS) aecm.h aecm_defines.h | dist
	EM_CACHE="$(CURDIR)/dist/emcache" $(EMCC) $(EMCPPFLAGS) $(EMCXXFLAGS) $(WASM_SRCS) -o $(WASM_OUT) $(EMLDFLAGS)

dist:
	mkdir -p dist

.PHONY: clean wasm
clean:
	# Object files and primary libraries (keep prebuilt libportaudio.a)
	rm -f *.o libaecm.a libaec3.a libaec3_*.a
	# Also remove nested object files in subdirectories
	find common_audio -name "*.o" -print -delete 2>/dev/null || true
	find modules -name "*.o" -print -delete 2>/dev/null || true
	find rtc_base -name "*.o" -print -delete 2>/dev/null || true
	find system_wrappers -name "*.o" -print -delete 2>/dev/null || true
	# Executables produced by this Makefile
	rm -f echoback cancel_file
	# Debug symbol bundles and temp files
	rm -rf *.dSYM
	rm -f *.tmp
	rm -rf dist
