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

Node* ExtractSequence(const std::string key, const YAML::Node& second) {
  if (second.size() == 1) {
    return new Sequence(key, EntreeType::STRING,
                        split(second[0].Scalar(), ' '));
  }
  std::vector<Node*> vector;
  for (const auto node : second) {
    if (node.IsMap()) {
      vector.push_back(LoadNode(node));
    } else {
      std::cerr << "ERRRRRROR" << std::endl;
    }
  }
  return new List(key, vector);
}

Structure* LoadNode(const YAML::Node& node) {
  std::vector<Node*> vector{};
  for (auto it = node.begin(); it != node.end(); ++it) {
    if (it->first.IsNull()) {
      std::cerr << "IsNull" << std::endl;
      continue;
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
        vector.push_back(ExtractSequence(key, second));
        break;

      case (YAML::NodeType::value::Map):
        vector.push_back(
          new Structure(key, LoadNode(second)->GetEntreeValues()));
        break;

      case (YAML::NodeType::value::Undefined):
        std::cerr << "Undefined" << std::endl;
        break;

      case (YAML::NodeType::value::Null):
        std::cerr << "Null" << std::endl;
    }
  }
  return new Structure("", vector);
}

Structure* LoadFile(const std::string& filename) {
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