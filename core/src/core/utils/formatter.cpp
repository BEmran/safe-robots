// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/formatter.hpp"

#include <sstream>

namespace core::utils {

Formatter::Formatter(const FormatFunc& func) : format_func_{func} {
}

std::string Formatter::Format(const LabeledModifier& lm,
                              const std::string& msg) const {
  return format_func_(lm, msg);
}

std::string NullFormatter(const LabeledModifier& /*lm*/,
                          const std::string& msg) {
  return msg;
}

std::string TimeLabelFormatter(const LabeledModifier& lm,
                               const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "][" << lm.GetLabel() << "] "
     << msg;
  return ss.str();
}

std::string TimeLabelModifierFormatter(const LabeledModifier& lm,
                                       const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]" << lm << " " << msg;
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
