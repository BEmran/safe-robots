#include <algorithm>
#include <dependencies/entree.hpp>
#include <iostream>

namespace yaml {
constexpr std::string_view TAP{"    "};
constexpr std::string_view HALF_TAP{"  "};

std::map<EntreeType, std::string> EntreeTypeStringMap{
  {EntreeType::INTEGER, "INTEGER"},
  {EntreeType::FLOAT, "FLOAT"},
  {EntreeType::STRING, "STRING"},
  {EntreeType::UNDEFINED, "UNDEFINED"}};

std::map<NodeType, std::string> NodeTypeStringMap{
  {NodeType::SEQUENCE, "SEQUENCE"},  //
  {NodeType::SINGLE, "SINGLE"},      //
  {NodeType::MAP, "MAP"},            //
  {NodeType::LIST, "LIST"},          //
  {NodeType::UNDEFINED, "UNDEFINED"}};

std::string EntreeTypeToString(const EntreeType type) {
  return EntreeTypeStringMap[type];
}

std::string NodeTypeToString(const NodeType type) {
  return NodeTypeStringMap[type];
}

std::string Single::PrintImpl() const {
  return m_value;
}

std::string Sequence::PrintImpl() const {
  std::string str = "[";
  std::for_each(m_value.begin(), m_value.end(),
                [&str](const std::string& v) { str += v + " "; });
  str.back() = ']';
  return str;
}

template <NodeType TYPE>
std::string Structure<TYPE>::Print(const std::string& offset) const {
  if (Key().empty()) {
    return PrintImpl(offset);
  } else {
    const std::string new_offset = offset + TAP.data();
    const std::string header = offset + Key() + SEPERATOR.data() + "\n";
    return header + PrintImpl(new_offset);
  }
}

std::string Map::PrintImpl(const std::string& offset) const {
  std::string str;
  std::for_each(m_nodes.begin(), m_nodes.end(), [&str, offset](Node* node) {
    str += node->Print(offset) + '\n';
  });
  str.pop_back();  // remove last '\n'
  return str;
}

std::string List::PrintImpl(const std::string& offset) const {
  const auto offset_size = offset.size();
  std::string updated_offset = offset + HALF_TAP.data();
  std::string str;
  for (const auto node : m_nodes) {
    auto tmp = node->Print(updated_offset) + '\n';
    tmp[offset_size] = '-';  // place after the offset length of each list item
    str += tmp;
  }
  str.pop_back();  // remove last '\n'
  return str;
}
}  // namespace yaml