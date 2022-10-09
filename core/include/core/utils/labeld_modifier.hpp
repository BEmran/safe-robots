// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LABELED_MODIFIER_HPP_
#define CORE_UTILS_LABELED_MODIFIER_HPP_

#include <memory>
#include <string>
#include <string_view>

#include "core/utils/event_level.hpp"
#include "core/utils/modifier.hpp"

namespace core::utils {

class LabeledModifier {
 public:
  explicit LabeledModifier(EventLevel event);

  LabeledModifier(EventLevel event, std::string_view label);

  LabeledModifier(EventLevel event, const Modifier& modifier);

  LabeledModifier(EventLevel event, std::string_view label,
                  const Modifier& modifier);
  /**
   * @brief convert object to string by decoding modifier value and label string
   *
   * @return std::string string represent the object string
   */
  std::string ToString() const;

  /**
   * @brief Get the EventLevel value
   *
   * @return EventLevel object event level
   */
  EventLevel GetEventLevel() const;

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
  EventLevel event_;
  std::string label_;
  Modifier modifier_;
};

/**
 * @brief override the << operator which writes the label using the modifier
 * configuration
 *
 * @param os output stream
 * @param lm labeled modifier
 * @return std::ostream& reference to the output stream
 */
std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm);

LabeledModifier DebugLabeledModifier();

LabeledModifier ErrorLabeledModifier();

LabeledModifier InfoLabeledModifier();

LabeledModifier WarnLabeledModifier();

}  // namespace core::utils

#endif  // CORE_UTILS_LABELED_MODIFIER_HPP_
