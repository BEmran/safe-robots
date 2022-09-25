// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef DEPENDENCIES_ENTREE_HPP_
#define DEPENDENCIES_ENTREE_HPP_

#include <map>
#include <string>
#include <vector>

namespace yaml {

using Map = std::map<std::string, std::string>;

enum class EntreeType { DECIMAL, REAL, STRING, UNDEFINED };
enum class NodeType { STRUCT, SINGLE, LIST, UNDEFINED };

// struct Entree {
//   EntreeType type = EntreeType::UNDEFINED;
//   std::string str;
// };

std::string EntreeTypeToString(EntreeType type);
std::string NodeTypeToString(NodeType type);

class Node {
 public:
  Node(const std::string& key, const NodeType type,
       const EntreeType entree_type)
    : m_key(key), m_type(type), m_entree_type(entree_type) {
  }
  virtual ~Node() = default;

  std::string Key() const {
    return m_key;
  }

  NodeType Type() const {
    return m_type;
  }

  inline std::string Info() const {
    return "[" + NodeTypeToString(m_type) + ":" +
           EntreeTypeToString(m_entree_type) + "]: ";
    // return ": ";
  }

  virtual std::string ToString() const = 0;

  virtual std::string ToString(const std::string& header) const {
    return header + ToString();
  };

  inline EntreeType GetEntreeType() const {
    return m_entree_type;
  }

 private:
  std::string m_key;
  NodeType m_type = NodeType::UNDEFINED;
  EntreeType m_entree_type = EntreeType::UNDEFINED;
};

class Single : public Node {
 public:
  Single(const std::string& key, const EntreeType& entree_type,
         const std::string& value)
    : Node(key, NodeType::SINGLE, entree_type), m_value(value) {
  }
  ~Single() = default;

  inline std::string GetEntreeValue() const {
    return m_value;
  }

  std::string ToString() const override;

 private:
  std::string m_value;
};

class List : public Node {
 public:
  List(const std::string& key, const EntreeType& entree_type,
       const std::vector<std::string>& values)
    : Node(key, NodeType::LIST, entree_type), m_values(values) {
  }
  ~List() = default;

  inline std::vector<std::string> GetEntreeValues() const {
    return m_values;
  }

  std::string ToString() const override;

 private:
  std::vector<std::string> m_values;
};

class Structure : public Node {
 public:
  Structure(const std::string& key, const std::vector<Node*>& nodes)
    : Node(key, NodeType::STRUCT, EntreeType::UNDEFINED), m_nodes(nodes) {
  }
  ~Structure() = default;

  inline std::vector<Node*> GetEntreeValues() const {
    return m_nodes;
  }

  std::string ToString() const override;
  std::string ToString(const std::string& header) const override;

 private:
  std::vector<Node*> m_nodes;
};

class Tree : public Node {
 public:
  Tree(const std::vector<Node*>& nodes)
    : Node("", NodeType::STRUCT, EntreeType::UNDEFINED), m_nodes(nodes) {
  }
  ~Tree() = default;

  inline std::vector<Node*> GetEntreeValues() const {
    return m_nodes;
  }

  std::string ToString() const override;
  std::string ToString(const std::string& header) const override;

 private:
  std::vector<Node*> m_nodes;
};

}  // namespace yaml
#endif  // DEPENDENCIES_YAML_HPP_