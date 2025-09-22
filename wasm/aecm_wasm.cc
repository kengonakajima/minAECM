// Minimal C API wrapper for the fixed-point AECM implementation to compile with Emscripten.
// - 16 kHz mono, 64-sample blocks
// - Exposes create/destroy/reset, bypass toggles, process, delay accessor

#include <cstdint>
#include <cstdlib>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define KEEPALIVE EMSCRIPTEN_KEEPALIVE
#else
#define KEEPALIVE
#endif

#include "aecm.h"
#include "aecm_defines.h"

struct AecmHandle {};

extern "C" {

KEEPALIVE void* aecm_create() {
  static_assert(BLOCK_LEN == 64, "This wrapper assumes 64-sample blocks");
  static_assert(SAMPLE_RATE_HZ == 16000, "This wrapper assumes 16 kHz sample rate");
  auto* handle = new AecmHandle();
  InitAecm();
  return reinterpret_cast<void*>(handle);
}

KEEPALIVE void aecm_destroy(void* handle) {
  if (!handle) {
    return;
  }
  delete reinterpret_cast<AecmHandle*>(handle);
}

KEEPALIVE void aecm_reset(void* handle) {
  if (!handle) {
    return;
  }
  InitAecm();
}

KEEPALIVE int aecm_process(void* handle, const int16_t* farend64, const int16_t* nearend64, int16_t* out64) {
  if (!handle || !farend64 || !nearend64 || !out64) {
    return -1;
  }
  return ProcessBlock(farend64, nearend64, out64);
}

KEEPALIVE void aecm_set_bypass_supmask(void* /*handle*/, int enable) {
  SetBypassSupMask(enable);
}

KEEPALIVE void aecm_set_bypass_nlp(void* /*handle*/, int enable) {
  SetBypassNlp(enable);
}

KEEPALIVE int aecm_get_last_delay_blocks(void* /*handle*/) {
  return GetLastEstimatedDelay();
}

}
