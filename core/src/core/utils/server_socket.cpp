// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/server_socket.hpp"

namespace core::utils {
ServerSocket::ServerSocket(uint16_t port)
  : ready_(false)
  , port_(port)
  , client_sock_(-1)
  , node_(std::make_unique<Node>(CreateSystemNode("Socket")))
  , socket_(std::make_unique<Socket>()) {
  Create();
}

void ServerSocket::Create() {
  if (!socket_->Create()) {
    node_->GetLogger()->LogWarn("Could not create a socket.");
    return;
  }
  node_->GetLogger()->LogDebug("Create a socket.");
  Bind();
}

void ServerSocket::Bind() {
  if (!socket_->Bind(port_)) {
    node_->GetLogger()->LogWarn("Could not bind to port " +
                                std::to_string(port_));
    return;
  }
  node_->GetLogger()->LogDebug("Bound to port " + std::to_string(port_));
  Listen();
}

void ServerSocket::Listen() {
  node_->GetLogger()->LogDebug("Listening at port " + std::to_string(port_));

  if (!socket_->Listen()) {
    node_->GetLogger()->LogWarn("Could not listen to socket.");
    return;
  }
}

void ServerSocket::Accept() {
  if (!ready_) {
    std::tie<bool, int>(ready_, client_sock_) = socket_->Accept();
  }

  if (!ready_) {
    node_->GetLogger()->LogWarn("Could not accept socket.");
    return;
  }

  node_->GetLogger()->LogDebug("Socket is ready at port " +
                               std::to_string(port_));
}

const ServerSocket& ServerSocket::operator<<(const std::string& msg) const {
  if (!ready_) {
    node_->GetLogger()->LogWarn("Cannot write to socket. Server is not ready.");
  } else if (!socket_->Send(client_sock_, msg)) {
    node_->GetLogger()->LogWarn("Failed writing to socket.");
    ready_ = false;
  }

  return *this;
}

const ServerSocket& ServerSocket::operator>>(std::string& msg) const {
  if (!ready_) {
    node_->GetLogger()->LogWarn(
      "Cannot read from socket. Server is not ready.");
  } else if (!socket_->Recv(client_sock_, &msg)) {
    node_->GetLogger()->LogWarn("Failed reading from socket.");
    ready_ = false;
  }

  return *this;
}
}  // namespace core::utils
