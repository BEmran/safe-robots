// Implementation of the Socket class.

#include "core/utils/socket.hpp"

#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>

namespace {
constexpr auto kMaxConnection = 5;
}  // namespace

namespace core::utils
{
Socket::Socket() : sock_(-1)
{
  memset(&address_, 0, sizeof(address_));
}

Socket::~Socket()
{
  if (IsValid())
    ::close(sock_);
}

bool Socket::Create()
{
  sock_ = socket(AF_INET, SOCK_STREAM, 0);
  return SetSocketOpt();
}

bool Socket::SetSocketOpt()
{
  if (!IsValid())
  {
    return false;
  }

  // TIME_WAIT - argh
  int on = 1;
  const auto res = ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
                                (const char*)&on, sizeof(on));
  return res != -1;
}

bool Socket::Bind(const int port)
{
  if (!IsValid())
  {
    return false;
  }

  address_.sin_family = AF_INET;
  address_.sin_addr.s_addr = INADDR_ANY;
  address_.sin_port = htons(port);

  const auto res = ::bind(sock_, (struct sockaddr*)&address_, sizeof(address_));
  return res != -1;
}

bool Socket::Listen() const
{
  if (!IsValid())
  {
    return false;
  }

  const auto res = ::listen(sock_, kMaxConnection);
  return res != -1;
}

std::pair<bool, int> Socket::Accept()
{
  const int addr_length = sizeof(address_);
  const auto new_sock =
      ::accept(sock_, (sockaddr*)&address_, (socklen_t*)&addr_length);

  return {sock_ > 0, new_sock};
}

bool Socket::Send(const int client_sock, const std::string& msg) const
{
  const auto status = ::send(client_sock, msg.c_str(), msg.size(), MSG_NOSIGNAL);
  return status != -1;
}

int Socket::Recv(const int client_sock, std::string& msg) const
{
  const auto buf_size = msg.size() + 1;
  char buf[buf_size];
  memset(buf, 0, buf_size);
  const int status = ::recv(client_sock, buf, buf_size, 0);
  msg = "";

  if (status == -1)
  {
    return 0;
  }
  msg = buf;
  return status;
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

bool Socket::IsValid() const
{
  return sock_ != -1;
}

}  // namespace core::utils