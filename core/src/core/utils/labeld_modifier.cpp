// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/labeld_modifier.hpp"

#include <iostream>
#include <map>
#include <string>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace {
static const std::map<EventLevel, std::string> kEventLevelNameMap = {
  {EventLevel::INFO, "INFO"},
  {EventLevel::DEBUG, "DEBUG"},
  {EventLevel::WARN, "WARN"},
  {EventLevel::ERROR, "ERROR"}};
}  // namespace

std::string EventLevelToString(const EventLevel event) {
  auto it = kEventLevelNameMap.find(event);
  if (it == kEventLevelNameMap.end()) {
    throw Exception("Undefined event level.");
  }
  return it->second;
}

LabeledModifier::LabeledModifier(EventLevel event)
  : LabeledModifier(event, EventLevelToString(event)) {
}

LabeledModifier::LabeledModifier(EventLevel event, std::string_view label)
  : LabeledModifier(event, label, DefaultModifier()) {
}

LabeledModifier::LabeledModifier(EventLevel event, const Modifier& modifier)
  : LabeledModifier(event, EventLevelToString(event), modifier) {
}

LabeledModifier::LabeledModifier(EventLevel event, std::string_view label,
                                 const Modifier& modifier)
  : event_(event), label_(label), modifier_(modifier) {
}

EventLevel LabeledModifier::GetEventLevel() const {
  return event_;
}

std::string LabeledModifier::GetLabel() const {
  return label_;
}

Modifier LabeledModifier::GetModifier() const {
  return modifier_;
}

std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm) {
  return os << lm.GetModifier() << lm.GetLabel() << DefaultModifier();
}

LabeledModifier DebugLabeledModifier() {
  const Modifier modifier = DebugModifier();
  const auto event = EventLevel::DEBUG;
  return LabeledModifier(event, modifier);
}

LabeledModifier ErrorLabeledModifier() {
  const Modifier modifier = ErrorModifier();
  const auto event = EventLevel::ERROR;
  return LabeledModifier(event, modifier);
}

LabeledModifier InfoLabeledModifier() {
  const Modifier modifier = InfoModifier();
  const auto event = EventLevel::INFO;
  return LabeledModifier(event, modifier);
}

LabeledModifier WarnLabeledModifier() {
  const Modifier modifier = WarnModifier();
  const auto event = EventLevel::WARN;
  return LabeledModifier(event, modifier);
}

}  // namespace core::utils
