#include <yaml-cpp/yaml.h>  // cppcheck-suppress

#include <dependencies/entree.hpp>
#include <dependencies/yaml.hpp>

namespace yaml {
namespace {
Node* ExtractSingle(const std::string& key, const YAML::Node& second);
Node* ExtractSequence(const std::string& key, const YAML::Node& second);
Structure* Extract(const YAML::Node& node);
Structure* LoadNode(const YAML::Node& node);

Structure* LoadNode(const YAML::Node& node) {
  std::vector<Node*> vector{};
  for (auto it = node.begin(); it != node.end(); ++it) {
    if (it->first.IsNull()) {
      std ::cerr << "IsNull" << std::endl;
      continue;
    }

    auto first = it->first;
    const auto key = first.as<std::string>();
    auto second = it->second;
    switch (second.Type()) {
      case (YAML::NodeType::value::Scalar):
        vector.push_back(ExtractSingle(key, second));
        break;

      case (YAML::NodeType::value::Sequence):
        vector.push_back(ExtractSequence(key, second));
        break;

      case (YAML::NodeType::value::Map):
        vector.push_back(
          new Structure(key, LoadNode(second)->GetEntreeValues()));
        break;

      case (YAML::NodeType::value::Undefined):
        std::cerr << "Node of type Undefined detected" << std::endl;
        break;

      case (YAML::NodeType::value::Null):
        std::cerr << "Node of type Null detected" << std::endl;
        break;

      default:
        std::cerr << "Undefined Type" << std::endl;
    }
  }
  return new Structure("", vector);  // create Root node
}

Node* ExtractSingle(const std::string& key, const YAML::Node& second) {
  return new Single(key, EntreeType::STRING, second.as<std::string>());
}

Node* ExtractSequence(const std::string& key, const YAML::Node& second) {
  if (second.size() == 1) {
    return new Sequence(key, EntreeType::STRING,
                        Split(second[0].Scalar(), ' '));
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

Structure* Extract(const YAML::Node& node) {
  try {
    return LoadNode(node);
  } catch (const YAML::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return new Structure("", {});
}
}  // namespace

template <typename Out>
void Split(const std::string& s, char delim, Out result) {
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) {
    *result++ = item;
  }
}

std::vector<std::string> Split(const std::string& s, char delim) {
  std::vector<std::string> elems;
  Split(s, delim, std::back_inserter(elems));
  return elems;
}

Structure* LoadFile(const std::string& filename) {
  const auto node = YAML::LoadFile(filename);
  return Extract(node);
}

Structure* LoadConfig(const std::string& config) {
  const auto node = YAML::Load(config);
  return Extract(node);
}

void DumpFile(const std::string& filename, const Map& map) {
  YAML::Node node;
  for (auto it = map.begin(); it != map.end(); ++it) {
    node[it->first] = it->second;
  }

  std::ofstream out(filename);
  out << node;
}
}  // namespace yaml