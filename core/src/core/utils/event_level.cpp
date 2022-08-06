#include "core/utils/event_level.hpp"

#include <iostream>
#include <map>
#include <string>

#include "core/utils/exception.hpp"

namespace core
{
namespace utils
{
namespace
{
static const std::map<EventLevel::event_level_t, std::string>
    EVENT_LEVEL_NAME_MAP = {{EventLevel::event_level_t::EL_INFO, "INFO"},
                            {EventLevel::event_level_t::EL_DEBUG, "DEBUG"},
                            {EventLevel::event_level_t::EL_WARN, "WARN"},
                            {EventLevel::event_level_t::EL_ERROR, "ERROR"}};
}  // namespace

std::string event_level_to_string(const EventLevel::event_level_t event)
{
  auto it = EVENT_LEVEL_NAME_MAP.find(event);
  if (it == EVENT_LEVEL_NAME_MAP.end())
  {
    throw Exception("Undefined event level.");
  }
  return it->second;
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event)
  : LabeledModifier(event, event_level_to_string(event))
{
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const std::string& label)
  : LabeledModifier(event, label, default_modifier())
{
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const Modifier& modifier)
  : LabeledModifier(event, event_level_to_string(event), modifier)
{
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const std::string& label,
                                 const Modifier& modifier)
  : event_(event), label_(label), modifier_(modifier)
{
}

EventLevel::event_level_t LabeledModifier::get_event_level() const
{
  return event_;
}

std::string LabeledModifier::get_label() const
{
  return label_;
}

Modifier LabeledModifier::get_modifier() const
{
  return modifier_;
}

std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm)
{
  return os << lm.get_modifier() << "[" << lm.get_label() << "]"
            << default_modifier();
}

LabeledModifier debug_labeled_modifier()
{
  const Modifier modifier = debug_modifier();
  const auto event = EventLevel::event_level_t::EL_DEBUG;
  return LabeledModifier(event, modifier);
}

LabeledModifier error_labeled_modifier()
{
  const Modifier modifier = error_modifier();
  const auto event = EventLevel::event_level_t::EL_ERROR;
  return LabeledModifier(event, modifier);
}

LabeledModifier info_labeled_modifier()
{
  const Modifier modifier = info_modifier();
  const auto event = EventLevel::event_level_t::EL_INFO;
  return LabeledModifier(event, modifier);
}

LabeledModifier warn_labeled_modifier()
{
  const Modifier modifier = warn_modifier();
  const auto event = EventLevel::event_level_t::EL_WARN;
  return LabeledModifier(event, modifier);
}

}  // namespace utils
}  // namespace core
