// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <iostream>

#include "core/utils/node.hpp"

using core::utils::CreateNode;
using core::utils::Node;

int main() {
  std::string name1 = "colorful";
  std::string name2 = "default";

  core::utils::Node node1 = CreateNode(name1);
  core::utils::Node node2(name2);

  node1.LogDebug("this is a debug message");
  node1.LogInfo("this is a info message");
  node1.LogWarn("this is a warn message");
  // node1.LogError("this is a error message");
  LOG_ERROR(node1, "this is a error message");
  node2.LogDebug("this is a debug message");
  node2.LogInfo("this is a info message");
  node2.LogWarn("this is a warn message");
  // node2.LogError("this is a error message");

  return 0;
}
