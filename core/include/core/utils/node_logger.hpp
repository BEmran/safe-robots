// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_NODE_LOGGER_HPP_
#define CORE_UTILS_NODE_LOGGER_HPP_

#include <memory>
#include <string>

#include "core/utils/event_level.hpp"
#include "core/utils/logger.hpp"

namespace core::utils {
/**
 * @brief holds various LabeledModifier objects to be used inside the node when
 * logging messages
 *
 */
struct NodeLabeledModifiers {
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

class NodeLogger {
 public:
  /**
   * @brief Construct a new Node Logger object using a logger
   *
   * @param logger shared ptr to a logger object
   */
  explicit NodeLogger(std::shared_ptr<Logger> logger);

  /**
   * @brief Construct a new Node Logger object using a logger with specific
   * labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
  NodeLogger(std::shared_ptr<Logger> logger,
             NodeLabeledModifiers labeled_modifiers);

  virtual ~NodeLogger() = default;

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
   * @brief Set the Log Header object
   *
   * @param header header to be added to each logging message
   */
  void SetLogHeader(const std::string& header);

 protected:
  /**
   * @brief Logs a message with specific LabeledModifier
   * @details this function is also called internally by all log_* functions
   *
   * @param lm labeled modifier which defines event and its label
   * @param msg message to be logged
   */
  virtual void LogImpl(const LabeledModifier& lm, const std::string& msg) const;

 private:
  std::shared_ptr<Logger> logger_;
  NodeLabeledModifiers labeled_modifiers_;
  std::string header_ = "";
};

/**
 * @brief Create a new NodeLogger with typical settings for console and file
 * formatter
 *
 * @param name name of Exception factory header, also used to create logger
 * filename as "<name>_logger.txt"
 * @return std::shared_ptr<NodeLogger> NodeLogger shared_ptr object
 */
std::shared_ptr<NodeLogger> CreateNodeLogger(const std::string& name);

/**
 * @brief Create a System NodeLogger with a common logger
 *
 * @param header NodeLogger header, typically set to node name
 * @return std::shared_ptr<NodeLogger> NodeLogger shared_ptr object
 */
std::shared_ptr<NodeLogger> CreateSystemNodeLogger(const std::string& header);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
