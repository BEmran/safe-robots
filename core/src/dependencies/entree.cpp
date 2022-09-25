#include <dependencies/entree.hpp>

namespace yaml {

std::map<EntreeType, std::string> EntreeTypeStringMap{
  {EntreeType::DECIMAL, "DECIMAL"},
  {EntreeType::REAL, "REAL"},
  {EntreeType::STRING, "STRING"},
  {EntreeType::UNDEFINED, "UNDEFINED"}};

std::map<NodeType, std::string> NodeTypeStringMap{
  {NodeType::STRUCT, "STRUCT"},
  {NodeType::SINGLE, "SINGLE"},
  {NodeType::LIST, "LIST"},
  {NodeType::UNDEFINED, "UNDEFINED"}};

std::string EntreeTypeToString(const EntreeType type) {
  return EntreeTypeStringMap[type];
}

std::string NodeTypeToString(const NodeType type) {
  return NodeTypeStringMap[type];
}

std::string Single::ToString() const {
  return Key() + Info() + m_value;
}

std::string List::ToString() const {
  std::string str = Key() + Info() + "[";
  for (size_t i = 0; i < m_values.size(); i++) {
    str += m_values[i];
    if (i < m_values.size() - 1) {
      str += ", ";
    }
  }
  str += "]";
  return str;
}

std::string Structure::ToString() const {
  return ToString("");
}

std::string Structure::ToString(const std::string& header) const {
  std::string str = header + Key() + Info() + "\n";
  for (size_t i = 0; i < m_nodes.size(); i++) {
    str += m_nodes[i]->ToString(header + "  ");
    if (i < m_nodes.size() - 1) {
      str += '\n';
    }
  }
  return str;
}

std::string Tree::ToString() const {
  return ToString("");
}

std::string Tree::ToString(const std::string& header) const {
  std::string str;
  for (size_t i = 0; i < m_nodes.size(); i++) {
    if (header.empty()) {
      str += m_nodes[i]->ToString("");
    } else {
      str += m_nodes[i]->ToString(header + "  ");
    }
    if (i < m_nodes.size() - 1) {
      str += '\n';
    }
  }
  return str;
}

}  // namespace yaml