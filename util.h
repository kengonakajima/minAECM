#include <stddef.h>
#include <stdint.h>

#define WORD16_MAX 32767
#define WORD16_MIN -32768
#define WORD32_MAX (int32_t)0x7fffffff
#define WORD32_MIN (int32_t)0x80000000

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))

#define ABS_W16(a) (((int16_t)(a) >= 0) ? ((int16_t)(a)) : -((int16_t)(a)))
#define ABS_W32(a) (((int32_t)(a) >= 0) ? ((int32_t)(a)) : -((int32_t)(a)))

#define UMUL_32_16(a, b) ((uint32_t)((uint32_t)(a) * (uint16_t)(b)))
#define MUL_16_U16(a, b) ((int32_t)(int16_t)(a) * (uint16_t)(b))
#define MUL_16_16(a, b) ((int32_t)(((int16_t)(a)) * ((int16_t)(b))))
#define MUL_16_16_RSFT(a, b, c) (MUL_16_16(a, b) >> (c))
#define MUL_16_16_RSFT_WITH_ROUND(a, b, c) \
  ((MUL_16_16(a, b) + ((int32_t)1 << ((c) - 1))) >> (c))

#define SAT(a, b, c) ((b) > (a) ? (a) : (b) < (c) ? (c) : (b))
#define SHIFT_W32(x, c) ((c) >= 0 ? (x) * (1 << (c)) : (x) >> -(c))

int CountLeadingZeros32(uint32_t n);
int CountLeadingZeros64(uint64_t n);
int16_t SatW32ToW16(int32_t value32);
int32_t AddSatW32(int32_t a, int32_t b);
int32_t SubSatW32(int32_t a, int32_t b);
int16_t AddSatW16(int16_t a, int16_t b);
int16_t SubSatW16(int16_t a, int16_t b);
int16_t NormW32(int32_t a);
int16_t NormU32(uint32_t a);
int16_t NormW16(int16_t a);
int16_t GetSizeInBits(uint32_t n);
int32_t MulAccumW16(int16_t a, int16_t b, int32_t c);
int16_t MaxAbsValueW16C(const int16_t* vector, size_t length);
#define MaxAbsValueW16 MaxAbsValueW16C
uint32_t DivU32U16(uint32_t num, uint16_t den);
int32_t DivW32W16(int32_t num, int16_t den);
int32_t SqrtFloor(int32_t value);

enum { kMaxFFTOrder = 10 };

struct RealFFT {
  int order;
};

int RealForwardFFT(struct RealFFT* self,
                   const int16_t* real_data_in,
                   int16_t* complex_data_out);
int RealInverseFFT(struct RealFFT* self,
                   const int16_t* complex_data_in,
                   int16_t* real_data_out);

int ComplexFFT(int16_t vector[], int stages, int mode);
int ComplexIFFT(int16_t vector[], int stages, int mode);
void ComplexBitReverse(int16_t* __restrict complex_data, int stages);
