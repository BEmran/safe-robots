#include "navio/server_socket.hpp"
#include "navio/socket_exception.hpp"
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  std::cout << "running....\n";

  if (argc < 2)
  {
    fprintf(stderr, "ERROR, no port provided\n");
    exit(1);
  }

  const int port = atoi(argv[1]);
  try
  {
    // Create the socket
    ServerSocket server(port);

    while (true)
    {
      ServerSocket new_sock;
      server.Accept(new_sock);

      try
      {
        while (true)
        {
          std::string data;
          new_sock >> data;
          new_sock << data;
        }
      }
      catch (SocketException&)
      {
      }
    }
  }
  catch (SocketException& e)
  {
    std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
  }

  return 0;
}