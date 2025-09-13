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
  echo_control_mobile.cc \
  aecm_core.cc \
  aecm_core_c.cc \
  delay_estimator_wrapper.cc \
  delay_estimator.cc

AECM_C_SRCS= \
  ring_buffer.c \
  spl_sqrt_floor.c \
  auto_corr_to_refl_coef.c \
  auto_correlation.c \
  complex_bit_reverse.c \
  complex_fft.c \
  copy_set_operations.c \
  cross_correlation.c \
  division_operations.c \
  downsample_fast.c \
  energy.c \
  filter_ar_fast_q12.c \
  filter_ar.c \
  filter_ma_fast_q12.c \
  get_hanning_window.c \
  get_scaling_square.c \
  ilbc_specific_functions.c \
  levinson_durbin.c \
  lpc_to_refl_coef.c \
  min_max_operations.c \
  randomization_functions.c \
  real_fft.c \
  refl_coef_to_lpc.c \
  spl_init.c \
  spl_inl.c \
  spl_sqrt.c \
  sqrt_of_one_minus_x_squared.c \
  vector_scaling_operations.c

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
