#ifndef _STF_EXPLICIT_TYPES_H_
#define _STF_EXPLICIT_TYPES_H_

// This file defines wrappers around strings that are used to document
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

class LocalFrame {
 public:
  explicit LocalFrame(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};

class RefFrame {
 public:
  explicit RefFrame(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};

class From {
 public:
  explicit From(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};

class To {
 public:
  explicit To(const std::string& id);
  std::string id() const;

 private:
  const std::string& id_;
};

}  // namespace stf

#endif  // _STF_EXPLICIT_TYPES_H_
