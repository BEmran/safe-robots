// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef DEPENDENCIES_ENTREE_HPP_
#define DEPENDENCIES_ENTREE_HPP_

#include <map>
#include <string>
#include <vector>

namespace yaml {

using Map = std::map<std::string, std::string>;

enum class EntreeType {
  INTEGER,   // hole number
  FLOAT,     // decimal number
  STRING,    // not a number
  UNDEFINED  // non leaf node
};

enum class NodeType {
  SINGLE,    // a node hold a single value
  SEQUENCE,  // a node hold a vector of values
  LIST,      // a node hold a vector of repeated nodes
  STRUCT,    // a node hold a sub nodes
  UNDEFINED  // nodes should not have this
};

// convert Entree Type to string
std::string EntreeTypeToString(EntreeType type);

// convert Node Type to string
std::string NodeTypeToString(NodeType type);

/**
 * @brief
 * Each node should have a key, type and entree type
 */
class Node {
 public:
  Node(const std::string& key, const NodeType type) : m_key(key), m_type(type) {
  }
  virtual ~Node() = default;

  std::string Key() const {
    return m_key;
  }

  NodeType Type() const {
    return m_type;
  }

  inline std::string Info() const {
    // return "[" + NodeTypeToString(m_type) + "]: ";
    return ": ";
  }

  // to recursively print node with offset
  virtual std::string Print(const std::string& offset = "") const = 0;

 private:
  std::string m_key{};
  NodeType m_type = NodeType::UNDEFINED;
};

class Single : public Node {
 public:
  Single(const std::string& key, const EntreeType& entree_type,
         const std::string& value)
    : Node(key, NodeType::SINGLE), m_value(value), m_entree_type(entree_type) {
  }

  inline EntreeType GetEntreeType() const {
    return m_entree_type;
  }

  inline std::string GetEntreeValue() const {
    return m_value;
  }

  std::string Print(const std::string& offset = "") const override;

 private:
  std::string m_value;
  EntreeType m_entree_type = EntreeType::UNDEFINED;  // make it only for leaf
};

class Sequence : public Node {
 public:
  Sequence(const std::string& key, const EntreeType& entree_type,
           const std::vector<std::string>& value)
    : Node(key, NodeType::SEQUENCE)
    , m_value(value)
    , m_entree_type(entree_type) {
  }

  inline EntreeType GetEntreeType() const {
    return m_entree_type;
  }

  inline std::vector<std::string> GetEntreeValue() const {
    return m_value;
  }

  std::string Print(const std::string& offset = "") const override;

 private:
  std::vector<std::string> m_value;
  EntreeType m_entree_type = EntreeType::UNDEFINED;  // make it only for leaf
};

class Structure : public Node {
 public:
  explicit Structure(const std::vector<Node*>& nodes) : Structure("", nodes) {
  }
  Structure(const std::string& key, const std::vector<Node*>& nodes)
    : Node(key, NodeType::STRUCT), m_nodes(nodes) {
  }
  ~Structure() = default;

  inline std::vector<Node*> GetEntreeValues() const {
    return m_nodes;
  }

  std::string Print(const std::string& offset = "") const override;

 private:
  std::vector<Node*> m_nodes;
};

class List : public Node {
 public:
  List(const std::string& key, const std::vector<Node*>& nodes)
    : Node(key, NodeType::LIST), m_nodes(nodes) {
  }
  ~List() = default;

  inline std::vector<Node*> GetEntreeValues() const {
    return m_nodes;
  }

  std::string Print(const std::string& offset = "") const override;

 private:
  std::vector<Node*> m_nodes;
};

}  // namespace yaml
#endif  // DEPENDENCIES_YAML_HPP_