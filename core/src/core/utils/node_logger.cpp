// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node_logger.hpp"

namespace core::utils {

constexpr std::string_view kSystemName = "sys";

NodeLogger::NodeLogger(const Logger& logger)
  : NodeLogger(logger, LoggerLabeledModifiers()) {
}

NodeLogger::NodeLogger(const Logger& logger,
                       LoggerLabeledModifiers labeled_modifiers)
  : logger_{logger}, labeled_modifiers_{labeled_modifiers} {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger)
  : NodeLogger(logger, LoggerLabeledModifiers()) {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger,
                       LoggerLabeledModifiers labeled_modifiers)
  : logger_{*(logger.get())}, labeled_modifiers_{labeled_modifiers} {
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
  const std::string updated_msg = header_ + msg.data();
  return StreamLogger(logger_, lm, updated_msg);
}

void NodeLogger::SetLabel(std::string_view header) {
  using namespace std::literals;
  if (header.empty()) {
    return;
  }
  header_ = "["s + header.data() + "] ";
}

NodeLogger CreatNodeLoggerUsingSystemLogger(std::string_view header) {
  static Logger logger{CreateStreamAndFileLogger(kSystemName, std::cout)};
  NodeLogger node_logger(logger);
  node_logger.SetLabel(header);
  return node_logger;
}
}  // namespace core::utils
