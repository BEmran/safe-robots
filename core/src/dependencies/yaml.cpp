#include <yaml-cpp/yaml.h>  // cppcheck-suppress

#include <dependencies/entree.hpp>
#include <dependencies/yaml.hpp>

namespace yaml {
namespace {
Node* ExtractSingle(const std::string& key, const YAML::Node& second);
Node* ExtractSequence(const std::string& key, const YAML::Node& second);
Node* Extract(const YAML::Node& node);
Map* LoadNode(const YAML::Node& node);

Map* LoadNode(const YAML::Node& node) {
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
        vector.push_back(new Map(key, LoadNode(second)->GetEntreeValue()));
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
  return new Map(vector);  // create Root node
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

Node* Extract(const YAML::Node& node) {
  try {
    return LoadNode(node);
  } catch (const YAML::Exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return new Map({});
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

Node* LoadFile(const std::string& filename) {
  const auto node = YAML::LoadFile(filename);
  return Extract(node);
}

Node* LoadConfig(const std::string& config) {
  const auto node = YAML::Load(config);
  return Extract(node);
}

void Dump(const std::string& filename, Node* node) {
  const auto re_parsed_node = YAML::Load(node->Print());
  std::ofstream out(filename);
  out << re_parsed_node;
}
}  // namespace yaml