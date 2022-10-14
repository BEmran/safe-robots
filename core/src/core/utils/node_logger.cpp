// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/node_logger.hpp"

namespace core::utils {

// constexpr std::string_view system_name = "sys";

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

// void NodeLogger::SetLabeledModifier(NodeLabeledModifiers labeled_modifiers)
// {
//   // labeled_modifiers_ = labeled_modifiers;
// }

// NodeLogger::NodeLogger(std::shared_ptr<Logger> logger)
//   : NodeLogger(std::move(logger), NodeLabeledModifiers()) {
// }

// NodeLogger::NodeLogger(std::shared_ptr<Logger> logger,
//                        NodeLabeledModifiers labeled_modifiers)
//   : logger_(std::move(logger))
//   , labeled_modifiers_(std::move(labeled_modifiers)) {
// }

NestedLogger NodeLogger::Debug(std::string_view msg) const {
  LogImpl(labeled_modifiers_.debug, msg);
  return NestedLogger(logger_);
}

const Logger& NodeLogger::Error(std::string_view msg) const {
  LogImpl(labeled_modifiers_.error, msg);
  return logger_;
}

const Logger& NodeLogger::Fatal(std::string_view msg) const {
  LogImpl(labeled_modifiers_.fatal, msg);
  return logger_;
}

const Logger& NodeLogger::Info(std::string_view msg) const {
  LogImpl(labeled_modifiers_.info, msg);
  return logger_;
}

const Logger& NodeLogger::Warn(std::string_view msg) const {
  LogImpl(labeled_modifiers_.warn, msg);
  return logger_;
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

NodeLogger CreateNodeLogger(const std::string& name) {
  const auto logger = CreateFileAndConsoleLogger(name);
  return NodeLogger(logger);
}

NodeLogger CreateSystemNodeLogger(const std::string& header) {
  static Logger logger{CreateFileAndConsoleLogger("sys")};
  NodeLogger node_logger(logger);
  node_logger.SetHeader(header);
  return node_logger;
}
}  // namespace core::utils
