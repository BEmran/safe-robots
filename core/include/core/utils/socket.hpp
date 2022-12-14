// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SOCKET_HPP_
#define CORE_UTILS_SOCKET_HPP_

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>
#include <utility>

namespace core::utils {
/**
 * @brief Wrapper for C socket function. A socket is a channel to connect two ip
 * address
 *
 */
class Socket {
 public:
  /**
   * @brief Construct a new Socket object
   *
   */
  Socket();

  /**
   * @brief Destroy the Socket object
   *
   */
  virtual ~Socket();

  /**
   * @brief Create a new socket and set socket FD's option.
   *
   * @return true if success.
   * @return false otherwise.
   */
  bool Create();

  /**
   * @brief Bind the socket to the current IP address on port
   *
   * @param port port number.
   * @return true if success.
   * @return false otherwise.
   */
  bool Bind(uint16_t port);

  /**
   * @brief tells the socket to listen to the incoming connections. It places
   * all incoming connection into a backlog queue until Accept() call accepts
   * the connection.
   * @return true if success.
   * @return false otherwise.
   */
  bool Listen() const;

  /**
   * @brief Wait for a connection on socket FD.
   * @details returns a new socket file descriptor for the accepted connection.
   * So, the original socket file descriptor can continue to be used
   * for accepting new connections while the new socket file descriptor is used
   * for communicating with the connected client.
   *
   * @return std::pair<bool, int> accept status and new socket file descriptor
   */
  std::pair<bool, int> Accept();

  // // Client initialization
  // bool Connect(const std::string& host, int port);

  /**
   * @brief Send a string message to socket FD.
   *
   * @param client_sock client socket file descriptor
   * @param msg string message to be sent
   * @return true if msg is sent successfully
   * @return false otherwise
   */
  static bool Send(int client_sock, const std::string& msg);

  /**
   * @brief Read a msg from socket FD.
   *
   * @param client_sock client socket file descriptor
   * @param msg received message
   * @return int
   */
  static int Recv(int client_sock, std::string* msg);

  // void SetNonBlocking(bool block);

  /**
   * @brief Check if socket is valid to be used
   *
   * @return true is opened or available
   * @return false otherwise
   */
  bool IsValid() const;

 protected:
  /**
   * @brief Set the Socket Opt object
   *
   * @return true if setting option was successful
   * @return false otherwise
   */
  bool SetSocketOpt() const;

 private:
  int sock_{-1};
  sockaddr_in address_;
};
}  // namespace core::utils
#endif  // CORE_UTILS_SOCKET_HPP_
