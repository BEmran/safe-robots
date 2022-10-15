// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node_logger.hpp"

namespace core::utils {

constexpr std::string_view kSystemName = "sys";

NodeLogger::NodeLogger(const Logger& logger)
  : NodeLogger(logger, NodeLabeledModifiers()) {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger)
  : NodeLogger(logger, NodeLabeledModifiers()) {
}

NodeLogger::NodeLogger(const Logger& logger,
                       NodeLabeledModifiers labeled_modifiers)
  : logger_{logger}, labeled_modifiers_{labeled_modifiers} {
}

NodeLogger::NodeLogger(std::shared_ptr<Logger> logger,
                       NodeLabeledModifiers labeled_modifiers)
  : logger_{*(logger.get())}, labeled_modifiers_{labeled_modifiers} {
}

NestedLogger NodeLogger::Debug(std::string_view msg) const {
  LogImpl(labeled_modifiers_.debug, msg);
  return NestedLogger(logger_);
}

NestedLogger NodeLogger::Error(std::string_view msg) const {
  LogImpl(labeled_modifiers_.error, msg);
  return NestedLogger(logger_);
}

NestedLogger NodeLogger::Fatal(std::string_view msg) const {
  LogImpl(labeled_modifiers_.fatal, msg);
  return NestedLogger(logger_);
}

NestedLogger NodeLogger::Info(std::string_view msg) const {
  LogImpl(labeled_modifiers_.info, msg);
  return NestedLogger(logger_);
}

NestedLogger NodeLogger::Warn(std::string_view msg) const {
  LogImpl(labeled_modifiers_.warn, msg);
  return NestedLogger(logger_);
}

void NodeLogger::SetHeader(std::string_view header) {
  using namespace std::literals;
  if (header.empty()) {
    return;
  }
  header_ = "["s + header.data() + "] ";
}

void NodeLogger::LogImpl(const LabeledModifier& lm,
                         std::string_view msg) const {
  logger_.Log(lm, header_ + msg.data());
}

NodeLogger CreatNodeLoggerUsingSystemLogger(std::string_view header) {
  static Logger logger{CreateStreamAndFileLogger(kSystemName)};
  NodeLogger node_logger(logger);
  node_logger.SetHeader(header);
  return node_logger;
}
}  // namespace core::utils
