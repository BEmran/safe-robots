// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node.hpp"

namespace core::utils {
Node::Node(const std::string& name)
  : Node(name, std::make_shared<Logger>(name + "_logger.txt")) {
}

Node::Node(const std::string& name, std::shared_ptr<Logger> logger)
  : Node(name, std::move(logger), NodeLabeledModifiers()) {
}

Node::Node(const std::string& name, std::shared_ptr<Logger> logger,
           NodeLabeledModifiers labeled_modifiers)
  : name_(name)
  , logger_(std::move(logger))
  , labeled_modifiers_(std::move(labeled_modifiers)) {
}

void Node::LogDebug(const std::string& msg) const {
  Log(labeled_modifiers_.debug, msg);
}

void Node::LogError(const std::string& msg) const {
  Log(labeled_modifiers_.error, msg);
}

void Node::LogInfo(const std::string& msg) const {
  Log(labeled_modifiers_.info, msg);
}

void Node::LogWarn(const std::string& msg) const {
  Log(labeled_modifiers_.warn, msg);
}

std::string Node::GetName() const {
  return name_;
}

std::shared_ptr<const Logger> Node::GetLogger() const {
  return logger_;
}

void Node::Log(const LabeledModifier& lm, const std::string& msg) const {
  const std::string named_msg = "[" + name_ + "]: " + msg;
  logger_->Log(lm, named_msg);
}

Node CreateNode(const std::string& node_name) {
  const auto cformater = std::make_shared<DefaultFormater>(true);
  const auto fformater = std::make_shared<DefaultFormater>(false);
  const auto exception = std::make_shared<ExceptionFactory>("");
  const auto logger_name = node_name + "_logger.txt";
  const auto log =
    std::make_shared<Logger>(logger_name, fformater, cformater, exception);
  return Node(node_name, log);
}

Node CreateDefaultNode(const std::string& node_name) {
  static std::shared_ptr<Logger> log = nullptr;
  if (!log) {
    const auto cformater = std::make_shared<DefaultFormater>(true);
    const auto fformater = std::make_shared<DefaultFormater>(false);
    const auto exception = std::make_shared<ExceptionFactory>("");
    const char* logger_name = "sys_logger.txt";
    log =
      std::make_shared<Logger>(logger_name, fformater, cformater, exception);
  }
  return Node(node_name, log);
}
}  // namespace core::utils
