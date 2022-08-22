// Implementation of the Socket class.

#include "navio/socket.hpp"
#include "string.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>

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

  const auto res = ::listen(sock_, MAXCONNECTIONS);
  return res != -1;
}

bool Socket::Accept(Socket& new_socket) const
{
  const int addr_length = sizeof(address_);
  new_socket.sock_ =
      ::accept(sock_, (sockaddr*)&address_, (socklen_t*)&addr_length);

  return new_socket.sock_ > 0;
}

bool Socket::Send(const std::string& msg) const
{
  int status = ::send(sock_, msg.c_str(), msg.size(), MSG_NOSIGNAL);
  return status != -1;
}

int Socket::Recv(std::string& msg) const
{
  char buf[MAXRECV + 1];
  memset(buf, 0, MAXRECV + 1);
  const int status = ::recv(sock_, buf, MAXRECV, 0);
  msg = "";

  if (status == -1)
  {
    std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
    msg = "";
    return 0;
  }
  msg = buf;
  return status;
}

bool Socket::Connect(const std::string& host, const int port)
{
  if (!IsValid())
  {
    return false;
  }

  address_.sin_family = AF_INET;
  address_.sin_port = htons(port);

  int status = inet_pton(AF_INET, host.c_str(), &address_.sin_addr);

  if (errno == EAFNOSUPPORT)
  {
    return false;
  }

  status = ::connect(sock_, (sockaddr*)&address_, sizeof(address_));

  return status == 0;
}

void Socket::SetNonBlocking(const bool block)
{
  int opts = fcntl(sock_, F_GETFL);

  if (opts < 0)
  {
    return;
  }

  if (block)
  {
    opts = (opts | O_NONBLOCK);
  }
  else
  {
    opts = (opts & ~O_NONBLOCK);
  }

  fcntl(sock_, F_SETFL, opts);
}

bool Socket::IsValid() const
{
  return sock_ != -1;
}