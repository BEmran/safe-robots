// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/event_level.hpp"

#include <iostream>
#include <map>
#include <string>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace {
const std::map<EventLevel::event_level_t, std::string> kEventLevelNameMap = {
  {EventLevel::EL_INFO, "INFO"},
  {EventLevel::EL_DEBUG, "DEBUG"},
  {EventLevel::EL_WARN, "WARN"},
  {EventLevel::EL_ERROR, "ERROR"}};
}  // namespace

std::string EventLevelToString(const EventLevel::event_level_t event) {
  auto it = kEventLevelNameMap.find(event);
  if (it == kEventLevelNameMap.end()) {
    throw Exception("Undefined event level.");
  }
  return it->second;
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event)
  : LabeledModifier(event, EventLevelToString(event)) {
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const std::string& label)
  : LabeledModifier(event, label, DefaultModifier()) {
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const Modifier& modifier)
  : LabeledModifier(event, EventLevelToString(event), modifier) {
}

LabeledModifier::LabeledModifier(const EventLevel::event_level_t event,
                                 const std::string& label,
                                 const Modifier& modifier)
  : event_(event), label_(label), modifier_(modifier) {
}

EventLevel::event_level_t LabeledModifier::GetEventLevel() const {
  return event_;
}

std::string LabeledModifier::GetLabel() const {
  return label_;
}

Modifier LabeledModifier::GetModifier() const {
  return modifier_;
}

std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm) {
  return os << lm.GetModifier() << "[" << lm.GetLabel() << "]"
            << DefaultModifier();
}

LabeledModifier DebugLabeledModifier() {
  const Modifier modifier = DebugModifier();
  const auto event = EventLevel::EL_DEBUG;
  return LabeledModifier(event, modifier);
}

LabeledModifier ErrorLabeledModifier() {
  const Modifier modifier = ErrorModifier();
  const auto event = EventLevel::EL_ERROR;
  return LabeledModifier(event, modifier);
}

LabeledModifier InfoLabeledModifier() {
  const Modifier modifier = InfoModifier();
  const auto event = EventLevel::EL_INFO;
  return LabeledModifier(event, modifier);
}

LabeledModifier WarnLabeledModifier() {
  const Modifier modifier = WarnModifier();
  const auto event = EventLevel::EL_WARN;
  return LabeledModifier(event, modifier);
}

}  // namespace core::utils
