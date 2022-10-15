// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_NODE_LOGGER_HPP_
#define CORE_UTILS_NODE_LOGGER_HPP_

#include <memory>
#include <string>

#include "core/utils/labeld_modifier.hpp"
#include "core/utils/logger.hpp"

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
 * @brief holds various LabeledModifier objects to be used inside the node when
 * logging messages
 *
 */
struct NodeLabeledModifiers {
  LabeledModifier debug;
  LabeledModifier error;
  LabeledModifier fatal;
  LabeledModifier info;
  LabeledModifier warn;

  /**
   * @brief Construct a new Node Labeled Modifiers object
   *
   * @param debug_lm debug labeled modifier
   * @param error_lm error labeled modifier
   * @param fatal_lm fatal labeled modifier
   * @param info_lm info labeled modifier
   * @param warn_lm warn labeled modifier
   */
  NodeLabeledModifiers(const LabeledModifier& debug_lm,
                       const LabeledModifier& error_lm,
                       const LabeledModifier& fatal_lm,
                       const LabeledModifier& info_lm,
                       const LabeledModifier& warn_lm)
    : debug(debug_lm)
    , error(error_lm)
    , fatal(fatal_lm)
    , info(info_lm)
    , warn(warn_lm) {
  }

  /**
   * @brief Construct a new Node Labeled Modifiers object with default Labeled
   * Modifiers
   *
   */
  NodeLabeledModifiers()
    : NodeLabeledModifiers(DebugLabeledModifier(), ErrorLabeledModifier(),
                           FatalLabeledModifier(), InfoLabeledModifier(),
                           WarnLabeledModifier()) {
  }
};

/**
 * @brief A named logger uses LabeledModifier to log data
 *
 */
class NodeLogger {
 public:
  explicit NodeLogger(const Logger& logger);
  /**
   * @brief Construct a new Node Logger object using a logger with specific
   * labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
  NodeLogger(const Logger& logger, NodeLabeledModifiers labeled_modifiers);

  explicit NodeLogger(std::shared_ptr<Logger> logger);

  NodeLogger(std::shared_ptr<Logger> logger,
             NodeLabeledModifiers labeled_modifiers);

  /**
   * @brief Destroy the Node Logger object
   *
   */
  ~NodeLogger() = default;

  /**
   * @brief Log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   */
  NestedLogger Debug(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   */
  NestedLogger Error(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Fatal LabeledModifier
   *
   * @param msg msg to log
   */
  NestedLogger Fatal(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   */
  NestedLogger Info(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   */
  NestedLogger Warn(std::string_view msg = "") const;

  /**
   * @brief Set the Header string
   *
   * @param header header to be added to each logging message
   */
  void SetHeader(std::string_view header);

  /**
   * @brief Get the internal Logger object
   *
   * @return Logger& logger object
   */
  Logger& GetLogger();

 protected:
  /**
   * @brief Logs a message with specific LabeledModifier
   * @details this function is also called internally by all log_*
   functions
   *
   * @param lm labeled modifier which defines event and its label
   * @param msg message to be logged
   */
  void LogImpl(const LabeledModifier& lm, std::string_view msg) const;

 private:
  const Logger& logger_;
  NodeLabeledModifiers labeled_modifiers_;
  std::string header_{};
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
