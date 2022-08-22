// Implementation of the ServerSocket class

#include "core/utils/server_socket.hpp"
#include "core/utils/socket_exception.hpp"

ServerSocket::ServerSocket(int port)
{
  if (!Socket::Create())
  {
    throw SocketException("Could not create server socket.");
  }

  if (!Socket::Bind(port))
  {
    throw SocketException("Could not bind to port.");
  }

  if (!Socket::Listen())
  {
    throw SocketException("Could not listen to socket.");
  }
}

const ServerSocket& ServerSocket::operator<<(const std::string& msg) const
{
  if (!Socket::Send(msg))
  {
    throw SocketException("Could not write to socket.");
  }

  return *this;
}

const ServerSocket& ServerSocket::operator>>(std::string& msg) const
{
  if (!Socket::Recv(msg))
  {
    throw SocketException("Could not read from socket.");
  }

  return *this;
}

void ServerSocket::Accept(ServerSocket& ss)
{
  if (!Socket::Accept(ss))
  {
    throw SocketException("Could not accept socket.");
  }
}