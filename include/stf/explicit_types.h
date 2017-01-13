#ifndef _STF_EXPLICIT_TYPES_H_
#define _STF_EXPLICIT_TYPES_H_

// This file defines stf::Source and stf::Target, which exist solely to document
// arguments.

#include <string>

namespace stf {
class Source {
 public:
  explicit Source(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};

class Target {
 public:
  explicit Target(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};
}  // namespace stf

#endif  // _STF_EXPLICIT_TYPES_H_
