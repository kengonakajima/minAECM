#include "util.h"

#include <limits.h>
#include <stdlib.h>

int CountLeadingZeros32(uint32_t n) {
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

int CountLeadingZeros64(uint64_t n) {
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

int16_t SatW32ToW16(int32_t value32) {
  if (value32 > 32767) {
    return 32767;
  }
  if (value32 < -32768) {
    return -32768;
  }
  return (int16_t)value32;
}

int32_t AddSatW32(int32_t a, int32_t b) {
  const int32_t sum = (int32_t)((uint32_t)a + (uint32_t)b);
  if ((a < 0) == (b < 0) && (a < 0) != (sum < 0)) {
    return sum < 0 ? INT32_MAX : INT32_MIN;
  }
  return sum;
}

int32_t SubSatW32(int32_t a, int32_t b) {
  const int32_t diff = (int32_t)((uint32_t)a - (uint32_t)b);
  if ((a < 0) != (b < 0) && (a < 0) != (diff < 0)) {
    return diff < 0 ? INT32_MAX : INT32_MIN;
  }
  return diff;
}

int16_t AddSatW16(int16_t a, int16_t b) {
  return SatW32ToW16((int32_t)a + (int32_t)b);
}

int16_t SubSatW16(int16_t a, int16_t b) {
  return SatW32ToW16((int32_t)a - (int32_t)b);
}

int16_t NormW32(int32_t a) {
  if (a == 0) {
    return 0;
  }
  int32_t tmp = a < 0 ? ~a : a;
  return (int16_t)(CountLeadingZeros32((uint32_t)tmp) - 1);
}

int16_t NormU32(uint32_t a) {
  return a == 0 ? 0 : (int16_t)CountLeadingZeros32(a);
}

int16_t NormW16(int16_t a) {
  if (a == 0) {
    return 0;
  }
  int32_t a32 = a;
  int32_t tmp = a < 0 ? ~a32 : a32;
  return (int16_t)(CountLeadingZeros32((uint32_t)tmp) - 17);
}

int32_t MulAccumW16(int16_t a, int16_t b, int32_t c) {
  return a * b + c;
}

int16_t GetSizeInBits(uint32_t n) {
  return (int16_t)(32 - CountLeadingZeros32(n));
}

int16_t MaxAbsValueW16C(const int16_t* vector, size_t length) {
  int maximum = 0;
  for (size_t i = 0; i < length; ++i) {
    int absolute = abs((int)vector[i]);
    if (absolute > maximum) {
      maximum = absolute;
    }
  }
  if (maximum > 32767) {
    maximum = 32767;
  }
  return (int16_t)maximum;
}

uint32_t DivU32U16(uint32_t num, uint16_t den) {
  return den == 0 ? UINT32_MAX : (uint32_t)(num / den);
}

int32_t DivW32W16(int32_t num, int16_t den) {
  return den == 0 ? INT32_MAX : (int32_t)(num / den);
}

int32_t SqrtFloor(int32_t value) {
  if (value <= 0) {
    return 0;
  }
  int32_t root = 0;
  for (int i = 15; i >= 0; --i) {
    int32_t try1 = root + (1 << i);
    int32_t shifted = try1 << i;
    if (value >= shifted) {
      value -= shifted;
      root |= 2 << i;
    }
  }
  return root >> 1;
}
