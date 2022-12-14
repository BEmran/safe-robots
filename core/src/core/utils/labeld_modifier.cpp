// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/labeld_modifier.hpp"

#include <iostream>
#include <map>
#include <string>

#include "core/utils/exception.hpp"

namespace core::utils {

std::string LMToString(std::string_view label, const Modifier& modifier) {
  return modifier.ToString() + label.data() + DefaultModifier().ToString();
}

LabeledModifier::LabeledModifier(EventLevel event)
  : LabeledModifier(event, EventLevelToString(event)) {
}

LabeledModifier::LabeledModifier(EventLevel event, std::string_view label)
  : LabeledModifier(event, label, Modifier()) {
}

LabeledModifier::LabeledModifier(EventLevel event, const Modifier& modifier)
  : LabeledModifier(event, EventLevelToString(event), modifier) {
}

LabeledModifier::LabeledModifier(EventLevel event, std::string_view label,
                                 const Modifier& modifier)
  : event_{event}
  , label_{label}
  , modifier_{modifier}
  , lm_string_{LMToString(label, modifier)} {
}

const std::string& LabeledModifier::ToString() const {
  return lm_string_;
}

EventLevel LabeledModifier::GetEventLevel() const {
  return event_;
}

const std::string& LabeledModifier::GetLabel() const {
  return label_;
}

Modifier LabeledModifier::GetModifier() const {
  return modifier_;
}

std::ostream& operator<<(std::ostream& os, const LabeledModifier& lm) {
  return os << lm.ToString();
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

LabeledModifier FatalLabeledModifier() {
  const Modifier modifier = FatalModifier();
  const auto event = EventLevel::FATAL;
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
