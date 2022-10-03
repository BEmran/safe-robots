// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef DEPENDENCIES_ENTREE_HPP_
#define DEPENDENCIES_ENTREE_HPP_

#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace yaml {

class Node;
using NodeMap = std::map<std::string, Node*>;

namespace {
constexpr std::string_view SEPERATOR{": "};
}

enum class EntreeType {
  INTEGER,   // hole number
  FLOAT,     // decimal number
  STRING,    // not a number
  UNDEFINED  // non leaf node
};

enum class NodeType {
  LIST,      // a node hold a vector of values
  MAP,       // a node hold a single value
  SINGLE,    // a node hold a single value
  SEQUENCE,  // a node hold a vector of values
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
  Node(std::string_view key, const NodeType type) : m_key(key), m_type(type) {
  }
  virtual ~Node() = default;

  std::string Key() const {
    return m_key;
  }

  NodeType Type() const {
    return m_type;
  }

  // to recursively print node with offset
  virtual std::string Print(const std::string& offset = "") const = 0;

 private:
  std::string m_key;
  NodeType m_type = NodeType::UNDEFINED;
};

template <typename T>
class Leaf : public Node {
 public:
  // using PrintCB = std::function<std::string(const T&)>;
  Leaf(std::string_view key, NodeType type, const EntreeType entree_type,
       const T& value)
    : Node(key, type), m_value{value}, m_entree_type{entree_type} {
  }
  virtual ~Leaf() = default;

  inline EntreeType GetEntreeType() const {
    return m_entree_type;
  }

  inline T GetEntreeValue() const {
    return m_value;
  }

  inline void SetEntreeValue(const T& value) {
    m_value = value;
  }

  std::string Print(const std::string& offset = "") const override {
    return offset + Key() + SEPERATOR.data() + PrintImpl();
  }

 protected:
  virtual std::string PrintImpl() const = 0;
  T m_value;

 private:
  EntreeType m_entree_type = EntreeType::UNDEFINED;  // make it only for leaf
};

class Single : public Leaf<std::string> {
 public:
  Single(std::string_view key, const EntreeType entree_type,
         const std::string& value)
    : Leaf<std::string>(key, NodeType::SINGLE, entree_type, value){};
  ~Single() = default;

 protected:
  std::string PrintImpl() const override;
};

class Sequence : public Leaf<std::vector<std::string>> {
 public:
  Sequence(std::string_view key, const EntreeType entree_type,
           const std::vector<std::string>& value)
    : Leaf<std::vector<std::string>>(key, NodeType::SEQUENCE, entree_type,
                                     value){};
  ~Sequence() = default;

 protected:
  std::string PrintImpl() const override;
};

class Structure : public Node {
 public:
  Structure(std::string_view key, const NodeType type,
            const std::vector<Node*>& nodes)
    : Node(key, type), m_nodes{nodes} {
  }
  virtual ~Structure() = default;

  inline std::vector<Node*> GetEntreeValue() const {
    return m_nodes;
  }

  std::string Print(const std::string& offset = "") const override;

 protected:
  virtual std::string PrintImpl(const std::string& offset) const = 0;
  std::vector<Node*> m_nodes;
};

class Map : public Structure {
 public:
  explicit Map(const std::vector<Node*>& nodes) : Map("", nodes) {
  }
  Map(std::string_view key, const std::vector<Node*>& nodes)
    : Structure(key, NodeType::MAP, nodes) {
    InitializeNodeMap();
  }
  ~Map() = default;

  std::optional<const Node*> FindParent(const std::string& key) const;

  std::optional<Node*> operator[](const std::string& key) const;

  inline bool IsNodeMapValid() const {
    return m_node_map.size() == m_nodes.size();
  }

  inline NodeMap GetNodeMap() const {
    return m_node_map;
  }

 protected:
  void InitializeNodeMap();
  std::string PrintImpl(const std::string& offset) const override;

 private:
  NodeMap m_node_map;
};

class List : public Structure {
 public:
  List(std::string_view key, const std::vector<Node*>& nodes)
    : Structure(key, NodeType::LIST, nodes) {
  }
  ~List() = default;

 protected:
  std::string PrintImpl(const std::string& offset) const override;
};

}  // namespace yaml
#endif  // DEPENDENCIES_YAML_HPP_