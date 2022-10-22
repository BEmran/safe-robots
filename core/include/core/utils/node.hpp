// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_NODE_HPP_
#define CORE_UTILS_NODE_HPP_

#include <memory>
#include <string>
#include <string_view>

#include "core/utils/logger_node.hpp"

namespace core::utils {

class Node {
 public:
  /**
   * @brief Construct a new Node object using its name
   *
   * @param name node name
   */
  explicit Node(std::string_view name);

  /**
   * @brief Construct a new Node object using its name and logger
   *
   * @param name node name
   * @param logger shared ptr to a logger object
   */
  Node(std::string_view name, std::shared_ptr<NodeLogger> n_logger);

  /**
   * @brief Get the node's Name
   *
   * @return std::string node name
   */

  std::string GetName() const;

  /**
   * @brief Get a shared ptr to the NodeLogger
   *
   * @return shared_ptr to NodeLogger
   */
  std::shared_ptr<NodeLogger> GetNodeLogger() const;

 protected:
  std::string name_;
  std::shared_ptr<NodeLogger> n_logger_;
};

/**
 * @brief calls node's log_error function and pass to it error information
 *
 */
// define LOG_ERROR(node, msg)
// node.GetLogger()->Error(LOG_INFORMATION_STRING + ": " + (msg))

/**
 * @brief Create a Node with system Logger but with different node name
 *
 * @param node_name node name
 * @return Node object
 */
Node CreateNodeUsingSystemLogger(std::string_view node_name);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_HPP_
