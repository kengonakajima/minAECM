// Minimal Abseil string_view shim
#ifndef ABSL_STRINGS_STRING_VIEW_H_
#define ABSL_STRINGS_STRING_VIEW_H_

#include <string_view>

namespace absl {
using string_view = std::string_view;
}

#endif  // ABSL_STRINGS_STRING_VIEW_H_

