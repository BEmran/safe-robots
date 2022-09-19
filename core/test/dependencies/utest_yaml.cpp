// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <dependencies/yaml.hpp>
#include <fstream>
// #include <yaml-cpp/yaml.h>

TEST(Cash, SetAndGet) {
  auto map = yaml::LoadFile("config_in.yaml");
  yaml::DumpFile("config_out.yaml", map);
  // YAML::Node node1;
  // node1["file1"] = "file name one";
  // node1["file2"] = "file name two";
  // YAML::Node node2;
  // node2["username"] = "bara";
  // node2["password"] = "12345";
  // node2["filename"] = node1;
  // std::ofstream out("out.yaml");
  // out << node2;
}
