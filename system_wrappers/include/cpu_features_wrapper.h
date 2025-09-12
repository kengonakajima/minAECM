/* Minimal stub of cpu_features_wrapper.h for AECM-only build.
 * Provides no-op definitions sufficient for files that include it.
 */
#ifndef SYSTEM_WRAPPERS_INCLUDE_CPU_FEATURES_WRAPPER_H_
#define SYSTEM_WRAPPERS_INCLUDE_CPU_FEATURES_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* In this minimal build we don't detect CPU features. */
#ifndef WEBRTC_CPU_DETECTION
#define WEBRTC_CPU_DETECTION 0
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SYSTEM_WRAPPERS_INCLUDE_CPU_FEATURES_WRAPPER_H_

