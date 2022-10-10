// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/modifier.hpp"

#include <algorithm>
#include <string_view>

namespace core::utils {

namespace {
constexpr std::string_view Prefix = "\x1B[";
constexpr std::string_view Postfix = "m";
std::string OptionsToString(const std::vector<int>& options) {
  std::string str;
  std::for_each(options.begin(), options.end(), [&str](const int opt) {
    str += Prefix.data() + std::to_string(opt) + Postfix.data();
  });
  return str;
}
}  // namespace

Modifier::Modifier(const std::vector<int>& options)
  : options_string_{OptionsToString(options)} {
}

std::string Modifier::ToString() const {
  return options_string_;
}

std::ostream& operator<<(std::ostream& os, const Modifier& mod) {
  return os << mod.ToString();
}

Modifier DefaultModifier() {
  return Modifier({FG::DEFAULT, BG::DEFAULT, FMT::DEFAULT});
}

Modifier DebugModifier() {
  return Modifier({FG::LIGHT_BLUE, BG::DEFAULT, FMT::BOLD});
}

Modifier ErrorModifier() {
  return Modifier({FG::LIGHT_RED, BG::DEFAULT, FMT::BOLD});
}

Modifier FatalModifier() {
  return Modifier({FG::RED, BG::YELLOW, FMT::BOLD, FMT::BLINK});
}

Modifier InfoModifier() {
  return Modifier({FG::LIGHT_CYAN, BG::DEFAULT, FMT::BOLD});
}

Modifier WarnModifier() {
  return Modifier({FG::LIGHT_YELLOW, BG::DEFAULT, FMT::BOLD});
}

}  // namespace core::utils
