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
Socket::Socket() : sock_(-1) {
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
  const auto res = ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
                                reinterpret_cast<const char*>(&on), sizeof(on));
  return res != -1;
}

bool Socket::Bind(const uint16_t port) {
  if (!IsValid()) {
    return false;
  }

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(port);

  const auto res =
    ::bind(sock_, reinterpret_cast<sockaddr*>(&address_), sizeof(address_));
  return res != -1;
}

bool Socket::Listen() const {
  if (!IsValid()) {
    return false;
  }

  const auto res = ::listen(sock_, kMaxConnection);
  return res != -1;
}

std::pair<bool, int> Socket::Accept() {
  int addr_length = sizeof(address_);
  const auto new_sock = ::accept(sock_, reinterpret_cast<sockaddr*>(&address_),
                                 reinterpret_cast<socklen_t*>(&addr_length));

  return {sock_ > 0, new_sock};
}

bool Socket::Send(const int client_sock, const std::string& msg) {
  const auto status =
    ::send(client_sock, msg.c_str(), msg.size(), MSG_NOSIGNAL);
  return status != -1;
}

int Socket::Recv(const int client_sock, std::string* msg) {
  constexpr size_t kBufSize = 512;
  char buf[kBufSize];
  memset(buf, 0, kBufSize);
  const auto status = ::recv(client_sock, buf, kBufSize, 0);
  *msg = "";

  if (status == -1) {
    return 0;
  }
  *msg = buf;
  return static_cast<int>(status);
}

// bool Socket::Connect(const std::string& host, const int port)
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

// void Socket::SetNonBlocking(const bool block)
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
  return sock_ != -1;
}

}  // namespace core::utils
