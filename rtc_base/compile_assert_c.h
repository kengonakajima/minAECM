/* Minimal compile-time assert macros for C and C++. */
#ifndef RTC_BASE_COMPILE_ASSERT_C_H_
#define RTC_BASE_COMPILE_ASSERT_C_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define RTC_COMPILE_ASSERT(expr) _Static_assert((expr), "RTC_COMPILE_ASSERT")
#else
#define RTC_COMPILE_ASSERT(expr) typedef char rtc_compile_assert_failed[(expr) ? 1 : -1]
#endif

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // RTC_BASE_COMPILE_ASSERT_C_H_

