
#ifndef CORE_UTILS_NODE_HPP
#define CORE_UTILS_NODE_HPP

#include <memory>

#include "core/utils/event_level.hpp"
#include "core/utils/logger.hpp"

namespace core::utils {
/**
 * @brief holds various LabeledModifier objects to be used inside the node when
 * logging messages
 *
 */
struct NodeLabeledModifiers {
  using event_level_t = EventLevel::event_level_t;
  LabeledModifier debug;
  LabeledModifier error;
  LabeledModifier info;
  LabeledModifier warn;

  NodeLabeledModifiers(const LabeledModifier& debug_,
                       const LabeledModifier& error_,
                       const LabeledModifier& info_,
                       const LabeledModifier& warn_)
    : debug(debug_), error(error_), info(info_), warn(warn_) {
  }

  NodeLabeledModifiers()
    : NodeLabeledModifiers(DebugLabeledModifier(), ErrorLabeledModifier(),
                           InfoLabeledModifier(), WarnLabeledModifier()) {
  }
};

class Node {
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
   * labeled modifiers
   *
   * @param name node name
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers
   */
  Node(const std::string& name, std::shared_ptr<Logger> logger,
       NodeLabeledModifiers labeled_modifiers);

  /**
   * @brief log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   */
  void LogDebug(const std::string& msg) const;

  /**
   * @brief log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   */
  void LogError(const std::string& msg) const;

  /**
   * @brief log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   */
  void LogInfo(const std::string& msg) const;

  /**
   * @brief log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   */
  void LogWarn(const std::string& msg) const;

  /**
   * @brief Get the node's Name
   *
   * @return std::string node name
   */

  std::string GetName() const;

  /**
   * @brief Get a shared ptr to the node logger
   *
   * @return std::shared_ptr<const Logger>
   */
  std::shared_ptr<const Logger> GetLogger() const;

  /**
   * @brief Logs a message with specific LabeledModifier
   * @details this function is also called internally by all log_* functions
   *
   * @param lm labeled modifier which defines event and its label
   * @param msg message to be logged
   */
  void Log(const LabeledModifier& lm, const std::string& msg) const;

 protected:
  std::string name_;
  std::shared_ptr<Logger> logger_;
  NodeLabeledModifiers labeled_modifiers_;
};

/**
 * @brief calls node's log_error function and pass to it error information
 *
 */
#define LOG_ERROR(node, msg) node.LogError(LOG_INFORMATION_STRING + ": " + msg)

/**
 * @brief Create a Node with labeled modifier
 *
 * @param node_name node name
 * @return Node object
 */
Node CreateNode(const std::string& node_name);

/**
 * @brief Create a Node with a same logger
 *
 * @param node_name node name
 * @return Node object
 */
Node CreateDefaultNode(const std::string& node_name);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_HPP
