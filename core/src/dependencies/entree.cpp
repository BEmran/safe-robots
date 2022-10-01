#include <algorithm>
#include <dependencies/entree.hpp>
#include <iostream>

namespace yaml {
constexpr std::string_view TAP{"    "};
constexpr std::string_view HALF_TAP{"  "};
constexpr std::string_view SEPERATOR{": "};

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
  return offset + Key() + SEPERATOR.data() + m_value;
}

std::string Sequence::Print(const std::string& offset) const {
  std::string str = offset + Key() + SEPERATOR.data() + "[";
  std::for_each(m_value.begin(), m_value.end(),
                [&str](const std::string& value) { str += value + " "; });
  str.back() = ']';
  return str;
}

std::string Structure::Print(const std::string& offset) const {
  std::string str;
  std::string new_offset = offset;
  if (!Key().empty()) {
    str = offset + Key() + SEPERATOR.data() + "\n";
    new_offset += TAP.data();
  }
  str += PrintInternalNodes(new_offset);

  return str;
}

std::string Structure::PrintInternalNodes(const std::string& offset) const {
  std::string str;
  std::for_each(m_nodes.begin(), m_nodes.end(), [&str, offset](Node* node) {
    str += node->Print(offset) + '\n';
  });
  str.pop_back();  // remove last '\n'
  return str;
}

std::string List::Print(const std::string& offset) const {
  std::string str = offset + Key() + SEPERATOR.data() + "\n";
  std::string new_offset = offset + TAP.data() + HALF_TAP.data();
  str += PrintInternalNodes(new_offset);
  return str;
}

std::string List::PrintInternalNodes(const std::string& offset) const {
  std::string str;
  std::for_each(m_nodes.begin(), m_nodes.end(), [&str, offset](Node* node) {
    auto tmp = node->Print(offset) + '\n';
    tmp[4] = '-';  // place '-' at the beginning of each list item
    str += tmp;
  });
  str.pop_back();  // remove last '\n'
  return str;
}

}  // namespace yaml