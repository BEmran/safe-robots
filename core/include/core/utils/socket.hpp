// Definition of the Socket class

#ifndef Socket_class
#define Socket_class

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>

constexpr auto MAXHOSTNAME = 200;
constexpr auto MAXCONNECTIONS = 5;
constexpr auto MAXRECV = 500;

/**
 * @brief A socket is a channel to connect two ip address
 *
 */
class Socket
{
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
   * @brief Give the socket FD the local address.
   *
   * @param port port number.
   * @return true if success.
   * @return false otherwise.
   */
  bool Bind(const int port);

  /**
   * @brief Prepare to accept connections on socket FD.
   *
   * @return true if success.
   * @return false otherwise.
   */
  bool Listen() const;

  /**
   * @brief Wait for a connection on socket FD. When a connection arrives, open
   * a new socket to communicate with it. and update the socket number.
   *
   * @return true if success.
   * @return false otherwise.
   */
  bool Accept(Socket& new_socket) const;

  // Client initialization
  bool Connect(const std::string& host, const int port);

  /**
   * @brief Send a string message to socket FD.
   *
   * @param msg string message
   * @return true if msg is sent successfully
   * @return false otherwise
   */
  bool Send(const std::string& msg) const;

  /**
   * @brief Read a msg from socket FD.
   *
   * @param msg received message
   * @return int
   */
  int Recv(std::string& msg) const;

  void SetNonBlocking(const bool block);

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
  bool SetSocketOpt();

 private:
  int sock_;
  sockaddr_in address_;
};

#endif