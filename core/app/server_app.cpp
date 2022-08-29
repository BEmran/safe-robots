// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <iostream>
#include <string>

#include "core/utils/server_socket.hpp"

int main(int argc, char* argv[]) {
  auto app = core::utils::CreateDefaultNode("app");
  app.LogDebug("running....");

  if (argc < 2) {
    app.LogError("no port provided");
    return EXIT_FAILURE;
  }

  const auto port = static_cast<uint16_t>(atoi(argv[1]));

  // Create the socket
  core::utils::ServerSocket server(port);

  auto tries = 3;
  while (tries-- > 0) {
    server.Accept();
    while (server.IsReady()) {
      std::string data;
      server >> data;
      server << data;
    }
    app.LogWarn("Lost connection");
  }

  return 0;
}
