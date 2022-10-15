// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node.hpp"

namespace core::utils {
Node::Node(const std::string& name)
  : Node(name, CreatNodeLoggerUsingSystemLogger(name)) {
}

Node::Node(const std::string& name, const NodeLogger& n_logger)
  : name_(name), n_logger_(n_logger) {
}

std::string Node::GetName() const {
  return name_;
}

const NodeLogger& Node::GetLogger() const {
  return n_logger_;
}

Node CreateSystemNode(const std::string& node_name) {
  auto node_logger = CreatNodeLoggerUsingSystemLogger(node_name);
  return Node(node_name, node_logger);
}
}  // namespace core::utils
