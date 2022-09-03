// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_NODE_HPP_
#define CORE_UTILS_NODE_HPP_

#include <memory>
#include <string>

#include "core/utils/node_logger.hpp"

namespace core::utils {

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
  Node(const std::string& name, std::shared_ptr<NodeLogger> n_logger);

  /**
   * @brief Get the node's Name
   *
   * @return std::string node name
   */

  std::string GetName() const;

  /**
   * @brief Get a shared ptr to the NodeLogger
   *
   * @return std::shared_ptr<const NodeLogger>
   */
  std::shared_ptr<const NodeLogger> GetLogger() const;

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
  std::shared_ptr<NodeLogger> n_logger_;
};

/**
 * @brief calls node's log_error function and pass to it error information
 *
 */
#define LOG_ERROR(node, msg)                                                   \
  node.GetLogger().LogError(LOG_INFORMATION_STRING + ": " + (msg))

/**
 * @brief Create a Node with a same NodeLogger but with different node name
 *
 * @param node_name node name
 * @return Node object
 */
Node CreateSystemNode(const std::string& node_name);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_HPP_
