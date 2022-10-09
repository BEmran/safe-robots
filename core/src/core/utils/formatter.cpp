// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/formatter.hpp"

#include <sstream>

namespace core::utils {

Formatter::Formatter() : format_func_{NullFormatter} {
}

Formatter::Formatter(const FormatFunc& func) : format_func_{func} {
}

std::string Formatter::Format(const LabeledModifier& lm,
                              std::string_view msg) const {
  return format_func_(lm, msg);
}

std::string NullFormatter(const LabeledModifier& /*lm*/, std::string_view msg) {
  return msg.data();
}

std::string TimeLabelFormatter(const LabeledModifier& lm,
                               std::string_view msg) {
  return "[" + DateTime().TimeToString() + "][" + lm.GetLabel() + "] " +
         msg.data();
}

std::string TimeLabelModifierFormatter(const LabeledModifier& lm,
                                       std::string_view msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "][" << lm << "] " << msg;
  return ss.str();
}

Formatter CreateNullFormatter() {
  return Formatter(NullFormatter);
}

Formatter CreateTimeLabelFormatter() {
  return Formatter(TimeLabelFormatter);
}

Formatter CreateTimeLabelModifierFormatter() {
  return Formatter(TimeLabelModifierFormatter);
}

}  // namespace core::utils
