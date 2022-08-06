#include "core/utils/node.hpp"

namespace core
{
namespace utils
{
Node::Node(const std::string& name)
  : Node(name, std::make_shared<Logger>(name + "_logger.txt"))
{
}

Node::Node(const std::string& name, std::shared_ptr<Logger> logger)
  : Node(name, std::move(logger), NodeLabeledModifiers())
{
}

Node::Node(const std::string& name, std::shared_ptr<Logger> logger,
           const NodeLabeledModifiers& labeled_modifiers)
  : name_(name)
  , logger_(std::move(logger))
  , labeled_modifiers_(labeled_modifiers)
{
}

void Node::log_debug(const std::string& msg)
{
  log(labeled_modifiers_.debug, msg);
}

void Node::log_error(const std::string& msg)
{
  log(labeled_modifiers_.error, msg);
}

void Node::log_info(const std::string& msg)
{
  log(labeled_modifiers_.info, msg);
}

void Node::log_warn(const std::string& msg)
{
  log(labeled_modifiers_.warn, msg);
}

std::string Node::get_name() const
{
  return name_;
}

std::shared_ptr<const Logger> Node::get_logger() const
{
  return logger_;
}

void Node::log(const LabeledModifier& lm, const std::string& msg)
{
  const std::string named_msg = "[" + name_ + "]: " + msg;
  logger_->log(lm, named_msg);
}

Node create_node(const std::string& node_name)
{
  const auto console_formater = std::make_shared<utils::DefaultFormater>(true);
  const auto file_formater = std::make_shared<utils::DefaultFormater>(false);
  const auto exception = std::make_shared<utils::ExceptionFactory>("");
  const auto logger_name = node_name + "_logger.txt";
  const auto log = std::make_shared<utils::Logger>(logger_name, file_formater,
                                                   console_formater, exception);
  return utils::Node(node_name, log);
}
}  // namespace utils
}  // namespace core
