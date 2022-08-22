// Definition of the ServerSocket class

#ifndef ServerSocket_class
#define ServerSocket_class

#include "socket.hpp"

class ServerSocket : private Socket
{
 public:
  ServerSocket() {};

  explicit ServerSocket(int port);

  virtual ~ServerSocket() {};

  // Data Transmission
  const ServerSocket& operator<<(const std::string& msg) const;
  const ServerSocket& operator>>(std::string& msg) const;

/**
 * @brief Accept a client
 * 
 */
  void Accept(ServerSocket& ss);
};

#endif