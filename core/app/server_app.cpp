#include "core/utils/server_socket.hpp"
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
  auto app = core::utils::CreateDefaultNode("app");
  app.LogDebug("running....");

  if (argc < 2)
  {
    app.LogError("no port provided");
    return EXIT_FAILURE;
  }

  const int port = atoi(argv[1]);

  // Create the socket
  core::utils::ServerSocket server(port);

  auto tries = 3;
  while(tries-- > 0)
  {
    server.Accept();
    while (server.IsReady())
    {
        std::string data;
        server >> data;
        server << data;
    }
    app.LogWarn("Lost connection");
  }

  return 0;
}