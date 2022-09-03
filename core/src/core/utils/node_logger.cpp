// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node_logger.hpp"

namespace core::utils {
NodeLogger::NodeLogger(std::shared_ptr<Logger> logger)
  : NodeLogger(std::move(logger), NodeLabeledModifiers()) {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger,
                       NodeLabeledModifiers labeled_modifiers)
  : logger_(std::move(logger))
  , labeled_modifiers_(std::move(labeled_modifiers)) {
}

void NodeLogger::LogDebug(const std::string& msg) const {
  LogImpl(labeled_modifiers_.debug, msg);
}

void NodeLogger::LogError(const std::string& msg) const {
  LogImpl(labeled_modifiers_.error, msg);
}

void NodeLogger::LogInfo(const std::string& msg) const {
  LogImpl(labeled_modifiers_.info, msg);
}

void NodeLogger::LogWarn(const std::string& msg) const {
  LogImpl(labeled_modifiers_.warn, msg);
}

void NodeLogger::SetLogHeader(const std::string& header) {
  if (header.empty()) {
    return;
  }
  header_ = "[" + header + "]: ";
}

void NodeLogger::LogImpl(const LabeledModifier& lm,
                         const std::string& msg) const {
  logger_->Log(lm, header_ + msg);
}

std::shared_ptr<NodeLogger> CreateNodeLogger(const std::string& name) {
  const auto logger = CreateFileAndConsoleLogger(name);
  auto node_logger = std::make_shared<NodeLogger>(logger);
  return node_logger;
}

std::shared_ptr<NodeLogger> CreateSystemNodeLogger(const std::string& header) {
  static std::shared_ptr<Logger> logger = nullptr;
  if (!logger) {
    logger = CreateFileAndConsoleLogger("sys");
  }
  auto node_logger = std::make_shared<NodeLogger>(logger);
  node_logger->SetLogHeader(header);
  return node_logger;
}
}  // namespace core::utils
