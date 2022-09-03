// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node.hpp"

namespace core::utils {
Node::Node(const std::string& name) : Node(name, CreateNodeLogger(name)) {
}

Node::Node(const std::string& name, std::shared_ptr<NodeLogger> n_logger)
  : name_(name), n_logger_(std::move(n_logger)) {
}

std::string Node::GetName() const {
  return name_;
}

std::shared_ptr<const NodeLogger> Node::GetLogger() const {
  return n_logger_;
}

Node CreateSystemNode(const std::string& node_name) {
  auto logger = CreateSystemNodeLogger(node_name);
  return Node(node_name, logger);
}
}  // namespace core::utils
