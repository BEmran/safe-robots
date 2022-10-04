#include <algorithm>
#include <dependencies/entree.hpp>
#include <iostream>

namespace yaml {
constexpr std::string_view TAP{"    "};
constexpr std::string_view HALF_TAP{"  "};

std::map<EntreeType, std::string> EntreeTypeStringStructure{
  {EntreeType::INTEGER, "INTEGER"},
  {EntreeType::FLOAT, "FLOAT"},
  {EntreeType::STRING, "STRING"},
  {EntreeType::UNDEFINED, "UNDEFINED"}};

std::map<NodeType, std::string> NodeTypeStringStructure{
  {NodeType::SEQUENCE, "SEQUENCE"},  //
  {NodeType::SINGLE, "SINGLE"},      //
  {NodeType::MAP, "MAP"},            //
  {NodeType::LIST, "LIST"},          //
  {NodeType::UNDEFINED, "UNDEFINED"}};

std::string EntreeTypeToString(const EntreeType type) {
  return EntreeTypeStringStructure[type];
}

std::string NodeTypeToString(const NodeType type) {
  return NodeTypeStringStructure[type];
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

void Structure::InitializeNodeMap() {
  for (const auto node : m_nodes) {
    if (m_node_map.find(node->Key()) != m_node_map.end()) {
      std::cerr << "found duplicated entree key: \'" << node->Key()
                << "\'. Cannot create a valid NodeMap. Keeping first one"
                << std::endl;
    } else {
      m_node_map.insert({node->Key(), node});
    }
  }
}

std::optional<const Node*> Structure::FindParent(const std::string& key) const {
  if (m_node_map.find(key) != m_node_map.end()) {
    return this;
  }

  for (const auto node : m_nodes) {
    if (node->Type() != NodeType::MAP) {
      continue;
    }
    auto result = dynamic_cast<const Structure*>(node)->FindParent(key);
    if (result) {
      return result;
    }
  }
  return {};
}

std::optional<Node*> Structure::operator[](const std::string& key) const {
  auto it = m_node_map.find(key);
  if (it != m_node_map.end()) {
    return it->second;
  }
  return {};
}

std::string CompositeNode::Print(const std::string& offset) const {
  if (Key().empty()) {
    return PrintImpl(offset);
  } else {
    const std::string new_offset = offset + TAP.data();
    const std::string header = offset + Key() + SEPERATOR.data() + "\n";
    return header + PrintImpl(new_offset);
  }
}

std::string Structure::PrintImpl(const std::string& offset) const {
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