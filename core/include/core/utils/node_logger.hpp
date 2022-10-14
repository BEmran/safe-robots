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

  NodeLabeledModifiers(const LabeledModifier& debug_,
                       const LabeledModifier& error_,
                       const LabeledModifier& fatal_,
                       const LabeledModifier& info_,
                       const LabeledModifier& warn_)
    : debug(debug_), error(error_), fatal(fatal_), info(info_), warn(warn_) {
  }

  NodeLabeledModifiers()
    : NodeLabeledModifiers(DebugLabeledModifier(), ErrorLabeledModifier(),
                           FatalLabeledModifier(), InfoLabeledModifier(),
                           WarnLabeledModifier()) {
  }
};

// void CreateNodeLabeledModifiers(const Modifier& modifier) {
//   NodeLabeledModifiers(LabeledModifier(EventLevel::DEBUG, modifier),
//                          LabeledModifier(EventLevel::ERROR, modifier),
//                          LabeledModifier(EventLevel::FATAL, modifier),
//                          LabeledModifier(EventLevel::INFO, modifier),
//                          LabeledModifier(EventLevel::WARN, modifier));
// }

class NodeLogger {
 public:
  explicit NodeLogger(const Logger& logger);
  explicit NodeLogger(std::shared_ptr<Logger> logger);
  /**
   * @brief Construct a new Node Logger object using a logger with specific
   * labeled modifiers
   *
   * @param logger shared ptr to a logger object
   * @param labeled_modifiers labeled modifier struct
   */
  NodeLogger(const Logger& logger, NodeLabeledModifiers labeled_modifiers);
  NodeLogger(std::shared_ptr<Logger> logger,
             NodeLabeledModifiers labeled_modifiers);

  ~NodeLogger() = default;
  /**
   * @brief log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   */
  const Logger& Debug(std::string_view msg = "") const;

  /**
   * @brief log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   */
  const Logger& Error(std::string_view msg = "") const;

  /**
   * @brief log the passed message using the Fatal LabeledModifier
   *
   * @param msg msg to log
   */
  const Logger& Fatal(std::string_view msg = "") const;

  /**
   * @brief log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   */
  const Logger& Info(std::string_view msg = "") const;

  /**
   * @brief log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   */
  const Logger& Warn(std::string_view msg = "") const;

  /**
   * @brief Set the Header object
   *
   * @param header header to be added to each logging message
   */
  void SetHeader(std::string_view header);

  Logger& GetLogger(const std::string& name);

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

  const Logger& logger_;
  NodeLabeledModifiers labeled_modifiers_;
  std::string header_ = "";
};

/**
 * @brief Create a new NodeLogger with typical settings for console and file
 * formatter
 *
 * @param name name of Exception factory header, also used to create logger
 * filename as "<name>_logger.txt"
 * @return NodeLogger shared_ptr object
 */
NodeLogger CreateNodeLogger(const std::string& name);

/**
 * @brief Create a System NodeLogger with a common logger
 *
 * @param header NodeLogger header, typically set to node name
 * @return NodeLogger shared_ptr object
 */
NodeLogger CreateSystemNodeLogger(const std::string& header);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
