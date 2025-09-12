# https://gist.github.com/azk-mochi/a36a53c9eb9a63bb5b70cfe4e288b560

PLATFORM=macosx
ARCH=arch -arm64

SDKROOT=$(shell xcrun --sdk $(PLATFORM) --show-sdk-path)
CXX=$(ARCH) $(shell xcrun --sdk $(PLATFORM) --find clang++)
CC=$(ARCH) $(shell xcrun --sdk $(PLATFORM) --find clang)
AR=$(shell xcrun --sdk $(PLATFORM) --find ar) 
RANLIB=$(shell xcrun --sdk $(PLATFORM) --find ranlib) 



INCFLAGS=-I. -isysroot $(SDKROOT)
# 不要なコンパイル時マクロは削除（ヘッダ側で必要定義は保持）
PFFLAGS=-DWEBRTC_POSIX
# サードパーティコードのビルド安定化のため、unused関連のWerrorは外す
WARN_CXX=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-private-field -Wunused-variable -Wno-unused-parameter
WARN_C=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-variable -Wno-unused-parameter -Wmissing-prototypes
CPPFLAGS=$(INCFLAGS) $(PFFLAGS)
CXXFLAGS=-std=c++20 -g -O3 -mmacosx-version-min=10.13 $(WARN_CXX)
CFLAGS=-std=c11 -g -O3 -mmacosx-version-min=10.13 $(WARN_C)


all: libaecm.a echoback cancel_file
	rm -f *.tmp


# ========================
# AECM ライブラリの構築
# ========================

# AECM に必要な最小ソース群（MIPS/NEON/テスト類は除外）
AECM_CC_SRCS= \
  modules/audio_processing/aecm/echo_control_mobile.cc \
  modules/audio_processing/aecm/aecm_core.cc \
  modules/audio_processing/aecm/aecm_core_c.cc \
  modules/audio_processing/utility/delay_estimator_wrapper.cc \
  modules/audio_processing/utility/delay_estimator.cc \
  rtc_base/checks.cc

AECM_C_SRCS= \
  common_audio/ring_buffer.c \
  common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c \
  common_audio/signal_processing/auto_corr_to_refl_coef.c \
  common_audio/signal_processing/auto_correlation.c \
  common_audio/signal_processing/complex_bit_reverse.c \
  common_audio/signal_processing/complex_fft.c \
  common_audio/signal_processing/copy_set_operations.c \
  common_audio/signal_processing/cross_correlation.c \
  common_audio/signal_processing/division_operations.c \
  common_audio/signal_processing/downsample_fast.c \
  common_audio/signal_processing/energy.c \
  common_audio/signal_processing/filter_ar_fast_q12.c \
  common_audio/signal_processing/filter_ar.c \
  common_audio/signal_processing/filter_ma_fast_q12.c \
  common_audio/signal_processing/get_hanning_window.c \
  common_audio/signal_processing/get_scaling_square.c \
  common_audio/signal_processing/ilbc_specific_functions.c \
  common_audio/signal_processing/levinson_durbin.c \
  common_audio/signal_processing/lpc_to_refl_coef.c \
  common_audio/signal_processing/min_max_operations.c \
  common_audio/signal_processing/randomization_functions.c \
  common_audio/signal_processing/real_fft.c \
  common_audio/signal_processing/refl_coef_to_lpc.c \
  common_audio/signal_processing/spl_init.c \
  common_audio/signal_processing/spl_inl.c \
  common_audio/signal_processing/spl_sqrt.c \
  common_audio/signal_processing/sqrt_of_one_minus_x_squared.c \
  common_audio/signal_processing/vector_scaling_operations.c

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
	$(CXX) -o echoback $(CXXFLAGS) $(CPPFLAGS) -I$(PORTAUDIO_HOME)/include -I$(PA_FALLBACK_INCDIR1) -I$(PA_FALLBACK_INCDIR2) -I$(PORTAUDIO_HOME) \
		echoback.cc libaecm.a \
		-L$(PORTAUDIO_HOME) -L$(PORTAUDIO_HOME)/lib -L$(PA_FALLBACK_LIBDIR1) -L$(PA_FALLBACK_LIBDIR2) \
		-Wl,-rpath,$(PORTAUDIO_HOME) -Wl,-rpath,$(PORTAUDIO_HOME)/lib -Wl,-rpath,$(PA_FALLBACK_LIBDIR1) -Wl,-rpath,$(PA_FALLBACK_LIBDIR2) \
		./libportaudio.a -framework CoreAudio -framework AudioToolbox -framework AudioUnit -framework CoreServices 

# Offline comparator (no PortAudio)
cancel_file: cancel_file.cc libaecm.a
	$(CXX) -o cancel_file $(CXXFLAGS) $(CPPFLAGS) cancel_file.cc libaecm.a

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

## wasm/emscripten ビルドは最小構成から削除
