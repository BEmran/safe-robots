// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node.hpp"

namespace core::utils {
Node::Node(std::string_view name)
  : Node(name, std::make_shared<NodeLogger>(
                 CreateNodeLoggerUsingSystemLogger(name))) {
}

Node::Node(std::string_view name, std::shared_ptr<NodeLogger> n_logger)
  : name_(name), n_logger_(n_logger) {
}

std::string Node::GetName() const {
  return name_;
}

std::shared_ptr<NodeLogger> Node::GetNodeLogger() const {
  return n_logger_;
}

Node CreateNodeUsingSystemLogger(std::string_view node_name) {
  auto node_logger =
    std::make_shared<NodeLogger>(CreateNodeLoggerUsingSystemLogger(node_name));
  return Node(node_name, node_logger);
}
}  // namespace core::utils
