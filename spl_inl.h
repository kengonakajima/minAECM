// This header file includes the inline functions in
// the fix point signal processing library.

static __inline int CountLeadingZeros32(uint32_t n) {
  if (n == 0) {
    return 32;
  }
  int count = 0;
  while ((n & 0x80000000u) == 0) {
    n <<= 1;
    ++count;
  }
  return count;
}

static __inline int CountLeadingZeros64(uint64_t n) {
  if (n == 0) {
    return 64;
  }
  int count = 0;
  while ((n & 0x8000000000000000ULL) == 0) {
    n <<= 1;
    ++count;
  }
  return count;
}

// Generic C implementation
static __inline int16_t SatW32ToW16(int32_t value32) {
  int16_t out16 = (int16_t)value32;

  if (value32 > 32767)
    out16 = 32767;
  else if (value32 < -32768)
    out16 = -32768;

  return out16;
}

static __inline int32_t AddSatW32(int32_t a, int32_t b) {
  // Do the addition in unsigned numbers, since signed overflow is undefined
  // behavior.
  const int32_t sum = (int32_t)((uint32_t)a + (uint32_t)b);

  // a + b can't overflow if a and b have different signs. If they have the
  // same sign, a + b also has the same sign iff it didn't overflow.
  if ((a < 0) == (b < 0) && (a < 0) != (sum < 0)) {
    // The direction of the overflow is obvious from the sign of a + b.
    return sum < 0 ? INT32_MAX : INT32_MIN;
  }
  return sum;
}

static __inline int32_t SubSatW32(int32_t a, int32_t b) {
  // Do the subtraction in unsigned numbers, since signed overflow is undefined
  // behavior.
  const int32_t diff = (int32_t)((uint32_t)a - (uint32_t)b);

  // a - b can't overflow if a and b have the same sign. If they have different
  // signs, a - b has the same sign as a iff it didn't overflow.
  if ((a < 0) != (b < 0) && (a < 0) != (diff < 0)) {
    // The direction of the overflow is obvious from the sign of a - b.
    return diff < 0 ? INT32_MAX : INT32_MIN;
  }
  return diff;
}

static __inline int16_t AddSatW16(int16_t a, int16_t b) {
  return SatW32ToW16((int32_t)a + (int32_t)b);
}

static __inline int16_t SubSatW16(int16_t var1, int16_t var2) {
  return SatW32ToW16((int32_t)var1 - (int32_t)var2);
}

static __inline int16_t GetSizeInBits(uint32_t n) {
  return 32 - CountLeadingZeros32(n);
}

// Return the number of steps a can be left-shifted without overflow,
// or 0 if a == 0.
static __inline int16_t NormW32(int32_t a) {
  return a == 0 ? 0 : CountLeadingZeros32(a < 0 ? ~a : a) - 1;
}

// Return the number of steps a can be left-shifted without overflow,
// or 0 if a == 0.
static __inline int16_t NormU32(uint32_t a) {
  return a == 0 ? 0 : CountLeadingZeros32(a);
}

// Return the number of steps a can be left-shifted without overflow,
// or 0 if a == 0.
static __inline int16_t NormW16(int16_t a) {
  const int32_t a32 = a;
  return a == 0 ? 0 : CountLeadingZeros32(a < 0 ? ~a32 : a32) - 17;
}

static __inline int32_t MulAccumW16(int16_t a, int16_t b, int32_t c) {
  return (a * b + c);
}
