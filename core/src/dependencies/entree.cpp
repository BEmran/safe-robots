#include <algorithm>
#include <dependencies/entree.hpp>
#include <iostream>

namespace yaml {
constexpr char TAP[] = "    ";
constexpr char HALF_TAP[] = "  ";

std::map<EntreeType, std::string> EntreeTypeStringMap{
  {EntreeType::INTEGER, "INTEGER"},
  {EntreeType::FLOAT, "FLOAT"},
  {EntreeType::STRING, "STRING"},
  {EntreeType::UNDEFINED, "UNDEFINED"}};

std::map<NodeType, std::string> NodeTypeStringMap{
  {NodeType::SEQUENCE, "SEQUENCE"},  //
  {NodeType::SINGLE, "SINGLE"},      //
  {NodeType::LIST, "LIST"},          //
  {NodeType::STRUCT, "STRUCT"},      //
  {NodeType::UNDEFINED, "UNDEFINED"}};

std::string EntreeTypeToString(const EntreeType type) {
  return EntreeTypeStringMap[type];
}

std::string NodeTypeToString(const NodeType type) {
  return NodeTypeStringMap[type];
}

std::string Single::Print(const std::string& offset) const {
  return offset + Key() + Info() + m_value;
}

std::string Sequence::Print(const std::string& offset) const {
  std::string str = offset + Key() + Info() + "[";
  std::for_each(m_value.begin(), m_value.end(),
                [&str](const std::string& value) { str += value + " "; });
  str.back() = ']';
  return str;
}

std::string Structure::Print(const std::string& offset) const {
  std::string str;
  std::string new_offset = offset;
  if (!Key().empty()) {
    str = offset + Key() + Info() + "\n";
    new_offset += TAP;
  }
  std::for_each(m_nodes.begin(), m_nodes.end(), [&str, new_offset](Node* node) {
    str += node->Print(new_offset) + '\n';
  });
  str.back() = ' ';

  return str;
}

std::string List::Print(const std::string& offset) const {
  std::string str = offset + Key() + Info() + "\n";
  const std::string new_offset = offset + TAP + HALF_TAP;
  for (size_t i = 0; i < m_nodes.size(); i++) {
    auto tmp = m_nodes[i]->Print(new_offset) + '\n';
    tmp[4] = '-';
    str += tmp;
  }
  str.back() = ' ';
  return str;
}

}  // namespace yaml