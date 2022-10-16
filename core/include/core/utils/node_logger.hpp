// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_NODE_LOGGER_HPP_
#define CORE_UTILS_NODE_LOGGER_HPP_

#include <memory>
#include <string>

#include "core/utils/labeld_modifier.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/logger_helper.hpp"

namespace core::utils {

/*
Public logger:
- using string:
Logger::Warn("....");
- using stream style:
Logger::Warn() << .... << endl;
- using printf style:
Logger::Warn(" %a %b %c", a, b, c) << .... << endl;

Private logger:
- create logger using logger name and filename for file writer
logger GetLogger(name, filename)
- create logger using logger name and formatter (style)
logger = GetLogger(name, write_formatter_pair)
- using string:
logger.Warn("....");
- using stream style:
logger.Warn() << .... << endl;
- using printf style:
logger.Warn(" %a %b %c", a, b, c) << .... << endl;

Inherit : public Logger
- using string:
this.Warn("....");
- using stream style:
this.Warn() << .... << endl;
- using printf style:
this.Warn(" %a %b %c", a, b, c) << .... << endl;
*/

/**
 * @brief A named logger uses LabeledModifier to log data
 *
 */
class NodeLogger {
 public:
  /**
   * @brief Construct a new Node Logger object using a logger with default
   * labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
  explicit NodeLogger(const Logger& logger);

  /**
   * @brief Construct a new Node Logger object using a logger with specific
   * labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
  NodeLogger(const Logger& logger, LoggerLabeledModifiers labeled_modifiers);

  explicit NodeLogger(std::shared_ptr<Logger> logger);

  NodeLogger(std::shared_ptr<Logger> logger,
             LoggerLabeledModifiers labeled_modifiers);

  /**
   * @brief Destroy the Node Logger object
   *
   */
  ~NodeLogger() = default;

  /**
   * @brief Log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Debug(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Error(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Fatal LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Fatal(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Info(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Warn(std::string_view msg = "") const;

  /**
   * @brief Set the Header string
   *
   * @param label a label to be added before logging message
   */
  void SetLabel(std::string_view label);

  /**
   * @brief Get the internal Logger object
   *
   * @return Logger& logger object
   */
  Logger& GetLogger();

 protected:
  /**
   * @brief Logs a message with specific LabeledModifier by calling internal
   * logger object
   *
   * @param lm labeled modifier which defines event and its label
   * @param msg message to be logged
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger LogMsg(const LabeledModifier& lm, std::string_view msg) const;

 private:
  const Logger& logger_;
  LoggerLabeledModifiers labeled_modifiers_;
  std::string label_{};
};

/**
 * @brief Create a NodeLogger using System Logger
 *
 * @param header NodeLogger header, typically set to node name
 * @return NodeLogger object
 */
NodeLogger CreatNodeLoggerUsingSystemLogger(std::string_view header);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
