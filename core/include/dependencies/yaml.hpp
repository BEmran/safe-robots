// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef DEPENDENCIES_YAML_HPP_
#define DEPENDENCIES_YAML_HPP_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "entree.hpp"

namespace yaml {

using Map = std::map<std::string, std::string>;

Structure* LoadFile(const std::string& filename);
Structure* LoadConfig(const std::string& config);
void DumpFile(const std::string& filename, const Map& map);

std::vector<std::string> Split(const std::string& s, char delim);

}  // namespace yaml
#endif  // DEPENDENCIES_YAML_HPP_