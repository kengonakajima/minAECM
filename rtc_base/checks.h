/* Minimal, absl非依存の CHECK 実装（教育用途の簡易版） */
#ifndef RTC_BASE_CHECKS_H_
#define RTC_BASE_CHECKS_H_

#include <stdio.h>
#include <stdlib.h>

/* DCHECK 有効判定 */
#if !defined(NDEBUG) || defined(DCHECK_ALWAYS_ON)
#define RTC_DCHECK_IS_ON 1
#else
#define RTC_DCHECK_IS_ON 0
#endif

/* noreturn 属性 */
#if defined(_MSC_VER)
#define RTC_NORETURN __declspec(noreturn)
#elif defined(__GNUC__)
#define RTC_NORETURN __attribute__((__noreturn__))
#else
#define RTC_NORETURN
#endif

/* 失敗時の終了ヘルパ（C/C++ 共通） */
static RTC_NORETURN inline void rtc_abort_with_message(const char* file,
                                                       int line,
                                                       const char* msg) {
  if (msg) {
    fprintf(stderr, "\nFatal CHECK failed at %s:%d: %s\n", file, line, msg);
  } else {
    fprintf(stderr, "\nFatal CHECK failed at %s:%d\n", file, line);
  }
  fflush(stderr);
  abort();
}

#ifdef __cplusplus
extern "C" {
#endif
/* C マクロ互換のためにエクスポート（実体は上のヘルパを呼ぶ） */
static RTC_NORETURN inline void rtc_FatalMessage(const char* file,
                                                int line,
                                                const char* msg) {
  rtc_abort_with_message(file, line, msg);
}
#ifdef __cplusplus
}
#endif

/* 基本マクロ */
#define RTC_CHECK(cond)                                                       \
  do {                                                                        \
    if (!(cond)) {                                                            \
      rtc_abort_with_message(__FILE__, __LINE__, "CHECK(" #cond ")");        \
    }                                                                         \
  } while (0)

#define RTC_CHECK_EQ(a, b) RTC_CHECK((a) == (b))
#define RTC_CHECK_NE(a, b) RTC_CHECK((a) != (b))
#define RTC_CHECK_LE(a, b) RTC_CHECK((a) <= (b))
#define RTC_CHECK_LT(a, b) RTC_CHECK((a) < (b))
#define RTC_CHECK_GE(a, b) RTC_CHECK((a) >= (b))
#define RTC_CHECK_GT(a, b) RTC_CHECK((a) > (b))

#define RTC_CHECK_NOTREACHED()                                                \
  do { rtc_abort_with_message(__FILE__, __LINE__, "NOTREACHED"); } while (0)

/* 便利関数（最小版） */
#ifdef __cplusplus
namespace rtc {
template <typename T>
inline T CheckedDivExact(T a, T b) {
  RTC_CHECK_EQ(a % b, 0);
  return a / b;
}
}  // namespace rtc
#endif

#endif  // RTC_BASE_CHECKS_H_
