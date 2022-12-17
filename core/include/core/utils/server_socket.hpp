// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SERVER_SOCKET_HPP_
#define CORE_UTILS_SERVER_SOCKET_HPP_

#include <memory>
#include <string>

#include "core/utils/node.hpp"
#include "core/utils/socket.hpp"

namespace core::utils {
class ServerSocket {
 public:
  /**
   * @brief Construct a new Server Socket object
   *
   * @param port port number
   */
  explicit ServerSocket(uint16_t port);

  /**
   * @brief Destroy the Server Socket object
   *
   */
  virtual ~ServerSocket() = default;

  // Data Transmission
  const ServerSocket& operator<<(const std::string& msg) const;
  const ServerSocket& operator>>(std::string& msg) const;

  /**
   * @brief Accept a new client to send and receive data and set client_sock
   * variable
   *
   */
  void Accept();

  /**
   * @brief indicate if socket is ready to send and receive
   *
   * @return true
   * @return false
   */
  inline bool IsReady() const {
    return ready_;
  }

 private:
  /**
   * @brief Crate socket by calling Socket.creat()
   *
   */
  void Create();

  /**
   * @brief Bind the socket by calling Socket.Bind()
   *
   */
  void Bind();

  /**
   * @brief Tells the socket to listen to the incoming connections by calling
   * Socket.Listen()
   *
   */
  void Listen();

  mutable bool ready_;          // ready to send and receive
  uint16_t port_;               // port number
  int client_sock_;             // client socket number set when call Accept()
  std::shared_ptr<Node> node_;  // node object
  std::unique_ptr<Socket> socket_;  // socket object
};
}  // namespace core::utils
#endif  // CORE_UTILS_SERVER_SOCKET_HPP_
