// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef DEPENDENCIES_YAML_HPP_
#define DEPENDENCIES_YAML_HPP_
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "entree.hpp"

namespace yaml {

using Map = std::map<std::string, std::string>;

Tree* LoadFile(const std::string& filename);
Tree* LoadNode(const YAML::Node& node);
void DumpFile(const std::string& filename, Map map);

}  // namespace yaml
#endif  // DEPENDENCIES_YAML_HPP_