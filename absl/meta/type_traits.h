// Minimal Abseil type_traits shim for rtc_base/sanitizer.h
#ifndef ABSL_META_TYPE_TRAITS_H_
#define ABSL_META_TYPE_TRAITS_H_

#include <type_traits>

namespace absl {

template <typename T>
using is_trivially_copy_constructible = std::is_trivially_copy_constructible<T>;

template <typename T>
using is_trivially_copy_assignable = std::is_trivially_copy_assignable<T>;

template <typename T>
using is_trivially_destructible = std::is_trivially_destructible<T>;

template <bool B, class T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <typename T>
using underlying_type_t = typename std::underlying_type<T>::type;

}  // namespace absl

#endif  // ABSL_META_TYPE_TRAITS_H_
