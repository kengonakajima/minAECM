// Minimal delay estimator wrapper API (fixed-size, single instance).

#include <stdint.h>

int InitDelayEstimatorFarend(void* handle);
int AddFarSpectrum(void* handle, const uint16_t* far_spectrum);
int InitDelayEstimator(void* handle);
int DelayEstimatorProcess(void* handle, const uint16_t* near_spectrum);
