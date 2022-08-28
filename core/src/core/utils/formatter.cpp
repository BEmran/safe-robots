// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/formatter.hpp"

#include <sstream>

namespace core::utils {
std::string NullFormatter::format(const LabeledModifier& lm,
                                  const std::string& msg) const {
  (void)lm;
  return msg;
}

DefaultFormater::DefaultFormater(const bool use_modifier)
  : use_modifier_(use_modifier) {
}

std::string DefaultFormater::format(const LabeledModifier& lm,
                                    const std::string& msg) const {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]";
  ss << AddLabeledModifier(lm);
  ss << ": " << msg;
  return ss.str();
}

std::string
DefaultFormater::AddLabeledModifier(const LabeledModifier& lm) const {
  std::stringstream ss;
  if (use_modifier_) {
    ss << lm;
  } else {
    ss << "[" << lm.GetLabel() << "]";
  }
  return ss.str();
}

}  // namespace core::utils
