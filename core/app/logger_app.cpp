// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <iostream>

#include "core/utils/node.hpp"

using core::utils::Node;

int main() {
  std::string name1 = "colorful";
  std::string name2 = "default";

  Node node1(name1);
  Node node2(name2);

  node1.GetLogger()->LogDebug("this is a debug message");
  node1.GetLogger()->LogInfo("this is a info message");
  node1.GetLogger()->LogWarn("this is a warn message");
  // node1.LogError("this is a error message");
  // LOG_ERROR(node1, "this is a error message");
  node2.GetLogger()->LogDebug("this is a debug message");
  node2.GetLogger()->LogInfo("this is a info message");
  node2.GetLogger()->LogWarn("this is a warn message");
  // node2.LogError("this is a error message");

  return 0;
}
