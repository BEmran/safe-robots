// Implementation of the ServerSocket class

#include "core/utils/server_socket.hpp"

namespace core::utils
{

ServerSocket::ServerSocket(const uint16_t port)
  : ready(false)
  , port_(port)
  , node_(std::make_unique<Node>(CreateDefaultNode("Socket")))
  , socket_(std::make_unique<Socket>())
{
  Create();
}

void ServerSocket::Create()
{
  if (!socket_->Create())
  {
    node_->LogWarn("Could not create a socket.");
    return;
  }
  node_->LogDebug("Create a socket.");
  Bind();
}

void ServerSocket::Bind()
{
  if (!socket_->Bind(port_))
  {
    node_->LogWarn("Could not bind to port " + std::to_string(port_));
    return;
  }
  node_->LogDebug("Bound to port " + std::to_string(port_));
  Listen();
}

void ServerSocket::Listen()
{
  node_->LogDebug("Listening at port " + std::to_string(port_));

  if (!socket_->Listen())
  {
    node_->LogWarn("Could not listen to socket.");
    return;
  }
}

void ServerSocket::Accept()
{
  if (!ready)
  {
    std::tie<bool, int>(ready, client_sock_) = socket_->Accept();
  }

  if (!ready)
  {
    node_->LogWarn("Could not accept socket.");
    return;
  }

  node_->LogDebug("Socket is ready at port " + std::to_string(port_));
}

const ServerSocket& ServerSocket::operator<<(const std::string& msg) const
{
  if (!ready)
  {
    node_->LogWarn("Cannot write to socket. Server is not ready.");
  }
  else if (!socket_->Send(client_sock_, msg))
  {
    node_->LogWarn("Failed writing to socket.");
    ready = false;
  }

  return *this;
}

const ServerSocket& ServerSocket::operator>>(std::string& msg) const
{
  if (!ready)
  {
    node_->LogWarn("Cannot read from socket. Server is not ready.");
  }
  else if (!socket_->Recv(client_sock_, msg))
  {
    node_->LogWarn("Failed reading from socket.");
    ready = false;
  }

  return *this;
}
}  // namespace core::utils