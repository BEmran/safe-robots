#include <dependencies/entree.hpp>

namespace yaml {

std::map<EntreeType, std::string> EntreeTypeStringMap{{DECIMAL, "DECIMAL"},
                                                      {REAL, "REAL"},
                                                      {STRING, "STRING"},
                                                      {UNDEFINED, "UNDEFINED"}};

std::map<NodeType, std::string> NodeTypeStringMap{{STRUCT, "STRUCT"},
                                                  {SINGLE, "SINGLE"},
                                                  {LIST, "LIST"},
                                                  {UNDEFINED, "UNDEFINED"}};

std::string EntreeTypeToString(EntreeType type) {
  return EntreeTypeStringMap[type];
}

std::string NodeTypeToString(NodeType type) {
  return NodeTypeStringMap[type];
}

std::string Single::ToString() const {
  return Key() + "[" + EntreeTypeToString(m_entree_type) + "]: " + m_value;
}

std::string List::ToString() const {
  std::string str = Key() + "[" + EntreeTypeToString(m_entree_type) + "]: [";
  for (size_t i = 0; i < m_values.size(); i++) {
    str += m_values[i];
    if (i < m_values.size() - 1) {
      str += ', ';
    }
  }
  return str;
}

std::string Structure::ToString() const {
  std::string str = Key() + ":\n";
  for (size_t i = 0; i < m_nodes.size(); i++) {
    str += m_nodes[i]->ToString();
    if (i < m_nodes.size() - 1) {
      str += '\n';
    }
  }
  return str;
}

}  // namespace yaml