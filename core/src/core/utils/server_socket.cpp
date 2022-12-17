// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/server_socket.hpp"

namespace core::utils {
ServerSocket::ServerSocket(uint16_t port)
  : ready_(false)
  , port_(port)
  , client_sock_(-1)
  , node_(std::make_shared<Node>(CreateNodeUsingSystemLogger("Server")))
  , socket_(std::make_unique<Socket>()) {
  Create();
}

void ServerSocket::Create() {
  if (!socket_->Create()) {
    node_->GetNodeLogger()->Warn("Could not create a socket.");
    return;
  }
  node_->GetNodeLogger()->Debug("Create a socket.");
  Bind();
}

void ServerSocket::Bind() {
  if (!socket_->Bind(port_)) {
    node_->GetNodeLogger()->Warn("Could not bind to port " +
                                 std::to_string(port_));
    return;
  }
  node_->GetNodeLogger()->Debug("Bound to port " + std::to_string(port_));
  Listen();
}

void ServerSocket::Listen() {
  node_->GetNodeLogger()->Debug("Listening at port " + std::to_string(port_));

  if (!socket_->Listen()) {
    node_->GetNodeLogger()->Warn("Could not listen to socket.");
    return;
  }
}

void ServerSocket::Accept() {
  if (ready_) {
    return;
  }
  std::optional<int> result = socket_->Accept();
  if (not result.has_value()) {
    node_->GetNodeLogger()->Warn("Could not accept socket.");
    return;
  }

  ready_ = true;
  client_sock_ = result.value();
  node_->GetNodeLogger()->Debug("Socket is ready at port: ") << port_;
}

const ServerSocket& ServerSocket::operator<<(const std::string& msg) const {
  if (!ready_) {
    node_->GetNodeLogger()->Warn(
      "Cannot write to socket. Server is not ready.");
  } else if (!socket_->Send(client_sock_, msg)) {
    node_->GetNodeLogger()->Warn("Failed writing to socket.");
    ready_ = false;
  }

  return *this;
}

const ServerSocket& ServerSocket::operator>>(std::string& msg) const {
  if (!ready_) {
    node_->GetNodeLogger()->Warn(
      "Cannot read from socket. Server is not ready.");
    return *this;
  }

  const auto msg_result = socket_->Recv(client_sock_);
  if (not msg_result.has_value()) {
    node_->GetNodeLogger()->Warn("Failed reading from socket.");
    ready_ = false;
  }

  msg = msg_result.value();
  return *this;
}
}  // namespace core::utils
