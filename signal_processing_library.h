/*
 * Minimal SPL header for minAECM (fixed-point utilities used by AECM).
 */

#ifndef COMMON_AUDIO_SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_LIBRARY_H_
#define COMMON_AUDIO_SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_LIBRARY_H_

#include <stdint.h>
#include <string.h>
#include <limits.h>

// Basic fixed-point constants/macros
#define WORD16_MAX 32767
#define WORD16_MIN -32768
#define WORD32_MAX (int32_t)0x7fffffff
#define WORD32_MIN (int32_t)0x80000000

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

// Absolute value helpers (note: edge cases documented in original SPL)
#define ABS_W16(a) (((int16_t)(a) >= 0) ? ((int16_t)(a)) : -((int16_t)(a)))
#define ABS_W32(a) (((int32_t)(a) >= 0) ? ((int32_t)(a)) : -((int32_t)(a)))

// Multiplication helpers
#define UMUL_32_16(a, b) ((uint32_t)((uint32_t)(a) * (uint16_t)(b)))
#define MUL_16_U16(a, b) ((int32_t)(int16_t)(a) * (uint16_t)(b))
#define MUL_16_16(a, b) ((int32_t)(((int16_t)(a)) * ((int16_t)(b))))
#define MUL_16_16_RSFT(a, b, c) (MUL_16_16(a, b) >> (c))
#define MUL_16_16_RSFT_WITH_ROUND(a, b, c) \
  ((MUL_16_16(a, b) + ((int32_t)1 << ((c) - 1))) >> (c))

// Saturate and shift helpers
#define SAT(a, b, c) ((b) > (a) ? (a) : (b) < (c) ? (c) : (b))
#define SHIFT_W32(x, c) ((c) >= 0 ? (x) * (1 << (c)) : (x) >> -(c))

#ifdef __cplusplus
extern "C" {
#endif

// Inline utilities (norm/sat/add etc.)
#include "spl_inl.h"

// Integer sqrt (used by magnitude path when ABS approximation未使用)
#include "spl_sqrt_floor.h"

// Min/Max (used: MaxAbsValueW16)
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length);
#define MaxAbsValueW16 MaxAbsValueW16C

// Division helpers used in AECM
uint32_t DivU32U16(uint32_t num, uint16_t den);
int32_t DivW32W16(int32_t num, int16_t den);

// FFT operations (minimal declarations)
int ComplexFFT(int16_t vector[], int stages, int mode);
int ComplexIFFT(int16_t vector[], int stages, int mode);
void ComplexBitReverse(int16_t* __restrict complex_data, int stages);

#ifdef __cplusplus
}
#endif

#endif  // COMMON_AUDIO_SIGNAL_PROCESSING_INCLUDE_SIGNAL_PROCESSING_LIBRARY_H_

