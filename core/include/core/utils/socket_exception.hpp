// SocketException class

#ifndef SocketException_class
#define SocketException_class

#include <string>

class SocketException
{
 public:
  explicit SocketException(const std::string& msg) : msg_(msg){};
  ~SocketException(){};

  std::string description()
  {
    return msg_;
  }

 private:
  std::string msg_;
};

#endif