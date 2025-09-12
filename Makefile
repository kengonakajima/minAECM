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
PFFLAGS=
WARN_CXX=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-private-field -Wunused-variable -Wno-unused-parameter \
          -Werror=unused-function -Werror=unused-variable -Werror=unused-private-field
WARN_C=-Wall -Wextra -Wunreachable-code -Wunused-function -Wunused-const-variable -Wunused-variable -Wno-unused-parameter -Wmissing-prototypes \
       -Werror=unused-function -Werror=unused-variable
CPPFLAGS=$(INCFLAGS) -std=c++20 $(PFFLAGS) -g -O3 -mmacosx-version-min=10.13 $(WARN_CXX)
CFLAGS=$(INCFLAGS) -std=c11 $(PFFLAGS) -g -O3 -mmacosx-version-min=10.13 $(WARN_C)


OBJS=ooura_fft.o

all: libaec3.a echoback cancel_file
	rm -f *.tmp



libaec3.a : $(OBJS)
	$(AR) cr libaec3.a $(OBJS)
	$(RANLIB) libaec3.a


PORTAUDIO_HOME?=$(HOME)/portaudio
PA_FALLBACK_INCDIR1=/opt/homebrew/opt/portaudio/include
PA_FALLBACK_INCDIR2=/usr/local/include
PA_FALLBACK_LIBDIR1=/opt/homebrew/opt/portaudio/lib
PA_FALLBACK_LIBDIR2=/usr/local/lib

# Echoback: port of echoback.js (local echo loop + AEC3)
echoback: echoback.cc libaec3.a libportaudio.a
	$(CXX) -o echoback $(CPPFLAGS) -I$(PORTAUDIO_HOME)/include -I$(PA_FALLBACK_INCDIR1) -I$(PA_FALLBACK_INCDIR2) -I$(PORTAUDIO_HOME) \
		echoback.cc libaec3.a \
		-L$(PORTAUDIO_HOME) -L$(PORTAUDIO_HOME)/lib -L$(PA_FALLBACK_LIBDIR1) -L$(PA_FALLBACK_LIBDIR2) \
		-Wl,-rpath,$(PORTAUDIO_HOME) -Wl,-rpath,$(PORTAUDIO_HOME)/lib -Wl,-rpath,$(PA_FALLBACK_LIBDIR1) -Wl,-rpath,$(PA_FALLBACK_LIBDIR2) \
		./libportaudio.a -framework CoreAudio -framework AudioToolbox -framework AudioUnit -framework CoreServices 

# Offline comparator (no PortAudio)
cancel_file: cancel_file.cc libaec3.a
	$(CXX) -o cancel_file $(CPPFLAGS) cancel_file.cc libaec3.a

.PHONY: clean wasm
clean:
	# Object files and primary libraries (keep prebuilt libportaudio.a)
	rm -f *.o libaec3.a libaec3_*.a
	# Executables produced by this Makefile
	rm -f echoback cancel_file
	# Debug symbol bundles and temp files
	rm -rf *.dSYM
	rm -f *.tmp

# Build WASM library for Node.js (outputs dist/aec3_wasm.js + .wasm)
EMXX?=em++
WASM_DIST=dist
WASM_JS=$(WASM_DIST)/aec3_wasm.js
WASM_SRCS=wasm/aec3_wasm.cc ooura_fft.cc

wasm:
	mkdir -p $(WASM_DIST)
	$(EMXX) -O3 -std=c++20 -I. $(WASM_SRCS) -o $(WASM_JS) \
	  -s MODULARIZE=1 -s EXPORT_NAME=AEC3Module -s ENVIRONMENT=node \
	  -s ALLOW_MEMORY_GROWTH=1 \
	  -s EXPORTED_FUNCTIONS='[_aec3_create,_aec3_destroy,_aec3_set_modes,_aec3_analyze,_aec3_process,_aec3_get_estimated_delay_blocks,_malloc,_free]' \
	  -s EXPORTED_RUNTIME_METHODS='[cwrap,ccall,HEAP16]'


# Cシム（demo/aec3.cpp）は撤去
# wavリーダ/ライタは最小構成では使用しない
# echo_canceller3_config は撤去
 

 
ooura_fft.o : ooura_fft.cc
	$(CXX) -c $(CPPFLAGS) ooura_fft.cc
 
 
 

 
 


 



# added webrtc 2022sep
 
 
