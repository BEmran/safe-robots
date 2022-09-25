#include <dependencies/entree.hpp>
#include <dependencies/yaml.hpp>

namespace yaml {

template <typename Out>
void split(const std::string& s, char delim, Out result) {
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) {
    *result++ = item;
  }
}

std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

const char* ValueToString(YAML::NodeType::value value) {
  switch (value) {
    case (YAML::NodeType::value::Map):
      return "Map";
    case (YAML::NodeType::value::Undefined):
      return "Undefined";
    case (YAML::NodeType::value::Scalar):
      return "Scalar";
    case (YAML::NodeType::value::Sequence):
      return "Sequence";
    case (YAML::NodeType::value::Null):
      return "Null";
  }
}

Tree* LoadNode(const YAML::Node& node) {
  std::cout << "size: " << node.size() << std::endl;
  std::vector<Node*> vector;
  for (auto it = node.begin(); it != node.end(); ++it) {
    if (it->first.IsNull()) {
      return nullptr;
    }
    auto first = it->first;
    const auto key = first.as<std::string>();
    auto second = it->second;
    std::vector<Node*> vec;
    switch (second.Type()) {
      case (YAML::NodeType::value::Scalar):
        vector.push_back(
          new Single(key, EntreeType::STRING, second.as<std::string>()));
        break;

      case (YAML::NodeType::value::Sequence):
        vec = std::vector<Node*>{};
        for (const auto node : second) {
          if (node.IsScalar()) {
            vector.push_back(
              new List(key, EntreeType::STRING, split(node.Scalar(), ' ')));
          } else if (node.IsMap()) {
            vec.push_back(LoadNode(node));
          } else {
            std::cerr << "ERRRRRROR" << std::endl;
          }
        }
        if (!vec.empty()) {
          vector.push_back(new Structure(key, vec));
        }
        break;

      case (YAML::NodeType::value::Map):
        vector.push_back(
          new Structure(key, LoadNode(second)->GetEntreeValues()));
        break;

      case (YAML::NodeType::value::Undefined):
        std::cerr << "Undefined" << std::endl;

      case (YAML::NodeType::value::Null):
        std::cerr << "Null" << std::endl;
    }
  }
  return new Tree(vector);
}

Tree* LoadFile(const std::string& filename) {
  const YAML::Node node = YAML::LoadFile(filename);
  return LoadNode(node);
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