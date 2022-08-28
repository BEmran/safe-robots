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
  const auto console_formater = std::make_shared<utils::DefaultFormater>(true);
  const auto file_formater = std::make_shared<utils::DefaultFormater>(false);
  const auto exception = std::make_shared<utils::ExceptionFactory>("");
  const auto logger_name = node_name + "_logger.txt";
  const auto log = std::make_shared<utils::Logger>(logger_name, file_formater,
                                                   console_formater, exception);
  return utils::Node(node_name, log);
}

Node CreateDefaultNode(const std::string& node_name) {
  const static auto console_formater =
    std::make_shared<utils::DefaultFormater>(true);
  const static auto file_formater =
    std::make_shared<utils::DefaultFormater>(false);
  const static auto exception = std::make_shared<utils::ExceptionFactory>("");
  const static auto logger_name = "sys_logger.txt";
  const static auto log = std::make_shared<utils::Logger>(
    logger_name, file_formater, console_formater, exception);

  return utils::Node(node_name, log);
}
}  // namespace core::utils
