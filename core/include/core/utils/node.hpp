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
  Node(const std::string& name, const NodeLogger& n_logger);

  /**
   * @brief Get the node's Name
   *
   * @return std::string node name
   */

  std::string GetName() const;

  /**
   * @brief Get a shared ptr to the NodeLogger
   *
   * @return const NodeLogger&
   */
  const NodeLogger& GetLogger() const;

 protected:
  std::string name_;
  NodeLogger n_logger_;
};

/**
 * @brief calls node's log_error function and pass to it error information
 *
 */
// #define LOG_ERROR(node, msg)                                                   \
//   node.GetLogger().Error(LOG_INFORMATION_STRING + ": " + (msg))

/**
 * @brief Create a Node with a same NodeLogger but with different node name
 *
 * @param node_name node name
 * @return Node object
 */
Node CreateSystemNode(const std::string& node_name);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_HPP_
