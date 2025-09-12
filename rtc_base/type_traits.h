// Minimal type_traits for rtc_base SafeCompare helpers.
#ifndef RTC_BASE_TYPE_TRAITS_H_
#define RTC_BASE_TYPE_TRAITS_H_

#include <type_traits>

namespace rtc {

// Treat integral types and enums as "int-like" for safe comparisons.
template <typename T>
struct IsIntlike
    : std::integral_constant<bool, std::is_integral<T>::value ||
                                       std::is_enum<T>::value> {};

}  // namespace rtc

#endif  // RTC_BASE_TYPE_TRAITS_H_

