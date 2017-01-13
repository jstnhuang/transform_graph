#include "stf/explicit_types.h"

#include <string>

namespace stf {
Source::Source(const std::string& id) : id_(id) {}
std::string Source::id() const { return id_; }

Target::Target(const std::string& id) : id_(id) {}
std::string Target::id() const { return id_; }
}  // namespace stf
