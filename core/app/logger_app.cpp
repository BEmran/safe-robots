// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <fstream>
#include <iostream>
#include <list>

#include "core/hardware/model.hpp"
#include "core/utils/node.hpp"

using core::utils::Node;

int main() {
  std::string name1 = "colorful";
  std::string name2 = "default";

  Node node1(name1);
  Node node2(name2);

  node1.GetNodeLogger()->Debug("this is a debug message");
  node1.GetNodeLogger()->Info("this is a info message");
  node1.GetNodeLogger()->Warn("this is a warn message");
  // node1.Error("this is a error message");
  // LOG_ERROR(node1, "this is a error message");
  node2.GetNodeLogger()->Debug("this is a debug message");
  node2.GetNodeLogger()->Info("this is a info message");
  node2.GetNodeLogger()->Warn("this is a warn message");
  // node2.Error("this is a error message");

  auto hardware_model = core::hardware::ExtractHardwareModel();
  node2.GetNodeLogger()->Debug(hardware_model.ToString());

  return 0;
}
