// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_EVENT_LEVEL_HPP_
#define CORE_UTILS_EVENT_LEVEL_HPP_

#include <memory>
#include <string>

#include "core/utils/terminal.hpp"

namespace core::utils {
/**
 * @brief Defines the level of an event to be deal with
 *
 */
struct EventLevel {
  typedef enum {
    EL_INFO = 0,  // basic level of information
    EL_DEBUG,     // only for information used in debug mode
    EL_WARN,      // when system behaviour is not desirable but not fatal
    EL_ERROR,     // when system is fatal and system should treat it
  } event_level_t;
};

class LabeledModifier {
 public:
  explicit LabeledModifier(EventLevel::event_level_t event);

  LabeledModifier(EventLevel::event_level_t event, const std::string& label);

  LabeledModifier(EventLevel::event_level_t event, const Modifier& modifier);

  LabeledModifier(EventLevel::event_level_t event, const std::string& label,
                  const Modifier& modifier);
  ~LabeledModifier() = default;

  EventLevel::event_level_t GetEventLevel() const;

  /**
   * @brief Returns the Modifier associated to an EventLevel
   *
   * @param event event level
   * @return terminal::Modifier* a pointer to associated the modifier
   */
  Modifier GetModifier() const;

  /**
   * @brief Returns the Modifier associated to an EventLevel
   *
   * @param event event level
   * @return terminal::Modifier* a pointer to associated the modifier
   */
  std::string GetLabel() const;

 private:
  EventLevel::event_level_t event_;
  std::string label_;
  Modifier modifier_;
};

/**
 * @brief Returns the name of the EventLevel
 *
 * @param event event level
 * @return std::string name of the event
 */
std::string EventLevelToString(EventLevel::event_level_t event);

/**
 * @brief override the << operator which writes the label using the modifier
 * configuration
 *
 * @param os output stream
 * @param lm labeled modifer
 * @return std::ostream& reference to the output stredam
 */
std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm);

LabeledModifier DebugLabeledModifier();

LabeledModifier ErrorLabeledModifier();

LabeledModifier InfoLabeledModifier();

LabeledModifier WarnLabeledModifier();

}  // namespace core::utils

#endif  // CORE_UTILS_EVENT_LEVEL_HPP_
