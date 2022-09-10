// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/terminal.hpp"

namespace core::utils {
using FG = terminal::FG;
using BG = terminal::BG;
using FMT = terminal::FMT;

namespace {
std::string SingleOptionToString(int option) {
  return "\x1B[" + std::to_string(option) + "m";
}
}  // namespace

Modifier::Modifier(FG fg, BG bg, FMT fmt) : fg_(fg), bg_(bg), fmt_(fmt) {
}

std::string Modifier::ToString() const {
  return SingleOptionToString(fmt_)    // change background
         + SingleOptionToString(fg_)   // change font format
         + SingleOptionToString(bg_);  // change foreground
}

std::ostream& operator<<(std::ostream& os, const Modifier& mod) {
  os << mod.ToString();
  return os;
}

Modifier DefaultModifier() {
  return Modifier(FG::FG_DEFAULT, BG::BG_DEFAULT, FMT::FMT_DEFAULT);
}

Modifier DebugModifier() {
  return Modifier(FG::FG_LIGHT_BLUE, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier ErrorModifier() {
  return Modifier(FG::FG_LIGHT_RED, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier InfoModifier() {
  return Modifier(FG::FG_LIGHT_CYAN, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier WarnModifier() {
  return Modifier(FG::FG_LIGHT_YELLOW, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

}  // namespace core::utils
