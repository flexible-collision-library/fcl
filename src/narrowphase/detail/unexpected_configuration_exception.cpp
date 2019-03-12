#include "fcl/narrowphase/detail/unexpected_configuration_exception.h"

#include <sstream>

namespace fcl {
namespace detail {

void ThrowUnexpectedConfigurationException(const std::string& message,
                                           const char* func,
                                           const char* file, int line) {
  std::stringstream ss;
  ss << file << ":(" << line << "): " << func << "(): " << message;
  throw UnexpectedConfigurationException(ss.str());
}

}  // namespace detail
}  // namespace fcl
