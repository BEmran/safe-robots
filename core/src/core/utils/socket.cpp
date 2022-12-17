// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/socket.hpp"

#include <fcntl.h>

#include <cerrno>
#include <cstring>
#include <iostream>

namespace {
constexpr auto kMaxConnection = 5;
}  // namespace

namespace core::utils {
Socket::Socket()
  : Socket(std::make_shared<Node>(CreateNodeUsingSystemLogger("Socket"))) {
}

Socket::Socket(std::shared_ptr<Node> node) : node_{node} {
  memset(&address_, 0, sizeof(address_));
}

Socket::~Socket() {
  if (IsValid()) {
    ::close(sock_);
  }
}

bool Socket::Create() {
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  return SetSocketOpt();
}

bool Socket::SetSocketOpt() const {
  if (!IsValid()) {
    return false;
  }

  // TIME_WAIT - argh
  int on = 1;
  const int res = ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
                               reinterpret_cast<const char*>(&on), sizeof(on));
  if (res == -1) {
    node_->GetNodeLogger()->Warn("Failed to set socket option")
      << std::strerror(errno);
    return false;
  }
  return true;
}

bool Socket::Bind(uint16_t port) {
  if (!IsValid()) {
    return false;
  }

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(port);

  const int res =
    ::bind(sock_, reinterpret_cast<sockaddr*>(&address_), sizeof(address_));
  if (res == -1) {
    node_->GetNodeLogger()->Warn("Failed to bind to connection")
      << std::strerror(errno);
    return false;
  }
  return true;
}

bool Socket::Listen() const {
  if (not IsValid()) {
    return false;
  }

  const int res = ::listen(sock_, kMaxConnection);
  return res != -1;
  if (res == -1) {
    node_->GetNodeLogger()->Warn("Failed to listen to connection")
      << std::strerror(errno);
    return false;
  }
  return true;
}

bool Socket::WaitForConnection() {
  if (not IsValid()) {
    return false;
  }
  node_->GetNodeLogger()->Info("Waiting for connection....");
  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  fd_set set;
  FD_ZERO(&set);
  FD_SET(sock_, &set);
  const int result = ::select(sock_ + 1, &set, NULL, NULL, &timeout);
  if (result == -1) {
    node_->GetNodeLogger()->Warn("Error occurred: ") << std::strerror(errno);
    return false;
  } else if (result == 0) {
    node_->GetNodeLogger()->Warn("Timeout occurred");
    return false;
  }
  return true;
}

std::optional<int> Socket::Accept() {
  if (not IsValid()) {
    return {};
  }

  if (not WaitForConnection()) {
    return {};
  }

  int addr_length = sizeof(address_);
  const int new_sock = ::accept(sock_, reinterpret_cast<sockaddr*>(&address_),
                                reinterpret_cast<socklen_t*>(&addr_length));
  if (new_sock == -1) {
    node_->GetNodeLogger()->Warn("Failed to accept connection")
      << std::strerror(errno);
    return {};
  }
  return new_sock;
}

bool Socket::Send(int client_sock, const std::string& msg) {
  const int status = ::send(client_sock, msg.c_str(), msg.size(), MSG_NOSIGNAL);
  if (status == -1) {
    node_->GetNodeLogger()->Warn("Failed to Send data") << std::strerror(errno);
    return {};
  }
  return true;
}

std::optional<std::string> Socket::Recv(int client_sock) {
  constexpr size_t kBufSize = 512;
  char buf[kBufSize];
  memset(buf, 0, kBufSize);
  const auto status = ::recv(client_sock, buf, kBufSize, 0);

  if (status == -1) {
    node_->GetNodeLogger()->Warn("Failed to Send data") << std::strerror(errno);
    return {};
  }
  return std::string{buf};
}

// bool Socket::Connect(const std::string& host, int port)
// {
//   if (!IsValid())
//   {
//     return false;
//   }

//   address_.sin_family = AF_INET;
//   address_.sin_port = htons(port);

//   int status = inet_pton(AF_INET, host.c_str(), &address_.sin_addr);

//   if (errno == EAFNOSUPPORT)
//   {
//     return false;
//   }

//   status = ::connect(sock_, (sockaddr*)&address_, sizeof(address_));

//   return status == 0;
// }

// void Socket::SetNonBlocking(bool block)
// {
//   int opts = fcntl(sock_, F_GETFL);

//   if (opts < 0)
//   {
//     return;
//   }

//   if (block)
//   {
//     opts = (opts | O_NONBLOCK);
//   }
//   else
//   {
//     opts = (opts & ~O_NONBLOCK);
//   }

//   fcntl(sock_, F_SETFL, opts);
// }

bool Socket::IsValid() const {
  if (sock_ == -1) {
    node_->GetNodeLogger()->Warn("Socket is not valid");
    return false;
  }
  return true;
}

}  // namespace core::utils
