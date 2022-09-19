#include <dependencies/yaml.hpp>

namespace yaml {

Map LoadFile(const std::string& filename) {
  const YAML::Node node = YAML::LoadFile(filename);
  return LoadNode(node);
}

Map LoadNode(const YAML::Node& node) {
  Map map;
  std::cout << "size: " << node.size() << std::endl;
  for (auto it = node.begin(); it != node.end(); ++it) {
    std::string key = it->first.as<std::string>();
    std::string value = " ";
    std::cout << "\t- ";
    if (node.Type() == YAML::NodeType::Sequence) {
      std::cout << " [Sequence] ";
      std::cout << std::quoted(key) << "\n";
    }
    if (node.Type() == YAML::NodeType::Map) {
      std::cout << " [map] ";
      try {
        value = it->second.as<std::string>();
        std::cout << std::quoted(key) << " is " << value << "\n";
        map.insert({key, value});
      } catch (...) {
        try {
          LoadNode(it->second.as<YAML::Node>());
        } catch (...) {
        }
      }
    }
  }
  return map;
}

void DumpFile(const std::string& filename, Map map) {
  YAML::Node node;
  for (auto it = map.begin(); it != map.end(); ++it) {
    node[it->first] = it->second;
  }

  std::ofstream out(filename);
  out << node;
}
}  // namespace yaml