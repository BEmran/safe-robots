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
 * @brief Logger Class used to log with stream method
 * @details When class is destructed it will log the stream data by calling the
 * logger function, this might throw an error depend on the the passed
 * LabeledModifier object
 */
class StreamLogger {
 public:
  // define end type for stream
  using endl_type = std::ostream&(std::ostream&);

  /**
   * @brief Construct a new Stream Logger object
   *
   * @param logger logger to use at destruction to log data
   * @param lm labeled modifier indicate level type and modifier
   * @param msg initial msg to log
   */
  StreamLogger(const Logger& logger, const LabeledModifier& lm,
               std::string_view ini_msg = "");

  /**
   * @brief Destroy the Stream Logger object
   *
   * @throws Logger::ExceptionFactory by calling the log function
   */
  ~StreamLogger() noexcept(false);

  /**
   * @brief template overriding stream operator
   *
   * @tparam T any type
   * @param obj object to be logged
   * @return StreamLogger& return reference of class to continue logging
   */
  template <typename T>
  StreamLogger& operator<<(T obj) {
    oss_ << obj;
    return *this;
  }

  /**
   * @brief to log endl object
   *
   */
  StreamLogger& operator<<(endl_type endl);

 private:
  /// @brief Reference to Logger used at destruction to log stream
  const Logger& logger_;
  /// @brief Labeled modifier used when calling Logger::Log function
  LabeledModifier lm_;
  /// @brief A stream to collect all data to be logged
  std::ostringstream oss_;
};

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
   * @param logger reference to Logger object
   */
  explicit NodeLogger(const Logger& logger);

  /**
   * @brief Construct a new Node Logger object using a logger with specific
   * labeled modifiers
   *
   * @param logger reference to Logger object
   * @param labeled_modifiers labeled modifier struct
   */
  NodeLogger(const Logger& logger, LoggerLabeledModifiers labeled_modifiers);

  /**
   * @brief Construct a new Node Logger object using a shared_ptr to logger with
   * default labeled modifiers
   *
   * @param logger shared ptr to a logger object
   */
  explicit NodeLogger(std::shared_ptr<Logger> logger);

  /**
   * @brief Construct a new Node Logger object using a shared_ptr to logger with
   * specific labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
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
  /// @brief Base reference to Logger object to log data
  const Logger& logger_;

  /// @brief List of labeled modifiers to use
  LoggerLabeledModifiers labeled_modifiers_;

  /// @brief Logger label
  std::string label_{};
};

/**
 * @brief Create a NodeLogger using System Logger
 *
 * @param header NodeLogger header, typically set to node name
 * @return NodeLogger object
 */
NodeLogger CreateNodeLoggerUsingSystemLogger(std::string_view header);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
