

#ifndef CORE_UTILS_EXCEPT_HPP
#define CORE_UTILS_EXCEPT_HPP

#include <string.h>

#include <stdexcept>

namespace core::utils
{
class Exception : public std::runtime_error
{
 public:
  explicit Exception(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

class ExceptionFactory
{
 public:
  explicit ExceptionFactory(const std::string& header) : header_(header)
  {
  }

  virtual ~ExceptionFactory()
  {
  }

  virtual void Throw(const std::string& msg) const
  {
    if (header_.empty())
    {
      throw Exception(msg);
    }
    throw Exception(header_ + ": " + msg);
  }

 private:
  std::string header_;
};

class NullExceptionFactory : public ExceptionFactory
{
 public:
  explicit NullExceptionFactory() : ExceptionFactory("")
  {
  }

  ~NullExceptionFactory()
  {
  }

  void Throw(const std::string& msg) const final
  {
    (void)msg;
  }
};

}  // namespace core::utils

#endif  // CORE_UTILS_EXCEPT_HPP