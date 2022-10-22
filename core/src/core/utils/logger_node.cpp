// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger_node.hpp"

namespace core::utils {

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger)
  : NodeLogger(logger, LoggerLabeledModifiers()) {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger,
                       LoggerLabeledModifiers labeled_modifiers)
  : logger_{logger}, labeled_modifiers_{labeled_modifiers} {
}

StreamLogger NodeLogger::Debug(std::string_view msg) const {
  return LogMsg(labeled_modifiers_.debug, msg);
}

StreamLogger NodeLogger::Error(std::string_view msg) const {
  return LogMsg(labeled_modifiers_.error, msg);
}

StreamLogger NodeLogger::Fatal(std::string_view msg) const {
  return LogMsg(labeled_modifiers_.fatal, msg);
}

StreamLogger NodeLogger::Info(std::string_view msg) const {
  return LogMsg(labeled_modifiers_.info, msg);
}

StreamLogger NodeLogger::Warn(std::string_view msg) const {
  return LogMsg(labeled_modifiers_.warn, msg);
}

StreamLogger NodeLogger::LogMsg(const LabeledModifier& lm,
                                std::string_view msg) const {
  const std::string updated_msg = label_ + msg.data();
  return StreamLogger(logger_, lm, updated_msg);
}

void NodeLogger::SetLabel(std::string_view header) {
  using namespace std::literals;
  if (header.empty()) {
    return;
  }
  label_ = "["s + header.data() + "] ";
}

NodeLogger CreateNodeLoggerUsingSystemLogger(std::string_view header) {
  auto logger = SystemLogger();
  NodeLogger node_logger(logger);
  node_logger.SetLabel(header);
  return node_logger;
}
}  // namespace core::utils
