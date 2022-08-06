
#ifndef CORE_UTILS_NODE_HPP
#define CORE_UTILS_NODE_HPP

#include <memory>

#include "core/utils/event_level.hpp"
#include "core/utils/logger.hpp"

namespace core
{
namespace utils
{
/**
 * @brief holds various LabeledModifier objects to be used inside the node when
 * logging messages
 *
 */
struct NodeLabeledModifiers
{
  using event_level_t = EventLevel::event_level_t;
  LabeledModifier debug;
  LabeledModifier error;
  LabeledModifier info;
  LabeledModifier warn;

  NodeLabeledModifiers(const LabeledModifier& debug_,
                       const LabeledModifier& error_,
                       const LabeledModifier& info_,
                       const LabeledModifier& warn_)
    : debug(debug_), error(error_), info(info_), warn(warn_)
  {
  }

  NodeLabeledModifiers()
    : NodeLabeledModifiers(debug_labeled_modifier(), error_labeled_modifier(),
                           info_labeled_modifier(), warn_labeled_modifier())
  {
  }
};

class Node
{
 public:
  /**
   * @brief Construct a new Node object using its name
   *
   * @param name node name
   */
  explicit Node(const std::string& name);

  /**
   * @brief Construct a new Node object using its name and logger
   *
   * @param name node name
   * @param logger shared ptr to a logger object
   */
  Node(const std::string& name, std::shared_ptr<Logger> logger);

  /**
   * @brief Construct a new Node object using its name, logger with specific
   * labeld modifiers
   *
   * @param name node name
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers
   */
  Node(const std::string& name, std::shared_ptr<Logger> logger,
       const NodeLabeledModifiers& labeled_modifiers);

  /**
   * @brief log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   */
  void log_debug(const std::string& msg);

  /**
   * @brief log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   */
  void log_error(const std::string& msg);

  /**
   * @brief log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   */
  void log_info(const std::string& msg);

  /**
   * @brief log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   */
  void log_warn(const std::string& msg);

  /**
   * @brief Get the node's Name
   *
   * @return std::string node name
   */

  std::string get_name() const;

  /**
   * @brief Get a shared ptr to the node logger
   *
   * @return std::shared_ptr<const Logger>
   */
  std::shared_ptr<const Logger> get_logger() const;

  /**
   * @brief Logs a message with specific LabeledModifier
   * @details this function is also called internally by all log_* functions
   *
   * @param lm labled modiefer which defines event and its label
   * @param msg message to be looged
   */
  void log(const LabeledModifier& lm, const std::string& msg);

 protected:
  std::string name_;
  std::shared_ptr<Logger> logger_;
  NodeLabeledModifiers labeled_modifiers_;
};

/**
 * @brief calls node's log_error fiunction and pass to it error information
 * 
 */
#define LOG_ERROR(node, msg) node.log_error(LOG_INFORMATION_STRING+ ": " + msg)

/**
 * @brief Create a Node with using labeld modifier
 * 
 * @param node_name node name
 * @return Node object
 */
Node create_node(const std::string& node_name);

}  // namespace utils
}  // namespace core

#endif  // CORE_UTILS_NODE_HPP
