#include "real_fft.h"

#include <stdlib.h>
#include <string.h>

#include "util.h"

// The C version FFT functions (i.e. RealForwardFFT and
// RealInverseFFT) are real-valued FFT wrappers for complex-valued
// FFT implementation in SPL.

int RealForwardFFT(struct RealFFT* self,
                             const int16_t* real_data_in,
                             int16_t* complex_data_out) {
  const int n = 1 << self->order;
  // The complex-value FFT implementation needs a buffer to hold 2^order
  // 16-bit COMPLEX numbers, for both time and frequency data.
  int16_t complex_buffer[2 << kMaxFFTOrder];

  // Insert zeros to the imaginary parts for complex forward FFT input.
  for (int i = 0, j = 0; i < n; i += 1, j += 2) {
    complex_buffer[j] = real_data_in[i];
    complex_buffer[j + 1] = 0;
  };

  ComplexBitReverse(complex_buffer, self->order);
  int result = ComplexFFT(complex_buffer, self->order, 1);

  // For real FFT output, use only the first N + 2 elements from
  // complex forward FFT.
  memcpy(complex_data_out, complex_buffer, sizeof(int16_t) * (n + 2));

  return result;
}

int RealInverseFFT(struct RealFFT* self,
                             const int16_t* complex_data_in,
                             int16_t* real_data_out) {
  const int n = 1 << self->order;
  // Create the buffer specific to complex-valued FFT implementation.
  int16_t complex_buffer[2 << kMaxFFTOrder];

  // For n-point FFT, first copy the first n + 2 elements into complex
  // FFT, then construct the remaining n - 2 elements by real FFT's
  // conjugate-symmetric properties.
  memcpy(complex_buffer, complex_data_in, sizeof(int16_t) * (n + 2));
  for (int i = n + 2; i < 2 * n; i += 2) {
    complex_buffer[i] = complex_data_in[2 * n - i];
    complex_buffer[i + 1] = -complex_data_in[2 * n - i + 1];
  }

  ComplexBitReverse(complex_buffer, self->order);
  int result = ComplexIFFT(complex_buffer, self->order, 1);

  // Strip out the imaginary parts of the complex inverse FFT output.
  for (int i = 0, j = 0; i < n; i += 1, j += 2) {
    real_data_out[i] = complex_buffer[j];
  }

  return result;
}
