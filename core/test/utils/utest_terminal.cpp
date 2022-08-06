#include <gtest/gtest.h>

#include "core/utils/terminal.hpp"
#include "utest/utils.hpp"

using namespace core::utils;

// check a modifier settings created using its constructor
TEST(Modifier, Construct)
{
  auto fg = FG::FG_DARK_GRAY;
  auto bg = BG::BG_LIGHT_YELLOW;
  auto fmt = FMT::FMT_BOLD;
  const Modifier modifier(fg, bg, fmt);
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check default modifier settings created using DefaultModifier function
TEST(Modifier, DefaultModifier)
{
  auto fg = FG::FG_DEFAULT;
  auto bg = BG::BG_DEFAULT;
  auto fmt = FMT::FMT_DEFAULT;
  const Modifier modifier = default_modifier();
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check debug modifier settings created using DebugModifier function
TEST(Modifier, DebugModifier)
{
  auto fg = FG::FG_LIGHT_BLUE;
  auto bg = BG::BG_DEFAULT;
  auto fmt = FMT::FMT_BOLD;
  const Modifier modifier = debug_modifier();
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check error modifier settings created using ErrorModifier function
TEST(Modifier, ErrorModifier)
{
  auto fg = FG::FG_LIGHT_RED;
  auto bg = BG::BG_DEFAULT;
  auto fmt = FMT::FMT_BOLD;
  const Modifier modifier = error_modifier();
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check info modifier settings created using InfoModifier function
TEST(Modifier, InfoModifier)
{
  auto fg = FG::FG_LIGHT_CYAN;
  auto bg = BG::BG_DEFAULT;
  auto fmt = FMT::FMT_BOLD;
  const Modifier modifier = info_modifier();
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check warning modifier settings created using WarnModifier function
TEST(Modifier, WarnModifier)
{
  auto fg = FG::FG_LIGHT_YELLOW;
  auto bg = BG::BG_DEFAULT;
  auto fmt = FMT::FMT_BOLD;
  Modifier modifier = warn_modifier();
  expect_eq_modifier(fg, bg, fmt, modifier);
}

// check overloading the << operator
TEST(Terminal, PrintOutModifier)
{
  auto fg = FG::FG_LIGHT_MAGENTA;
  auto bg = BG::BG_LIGHT_GRAY;
  auto fmt = FMT::FMT_BLINK;
  const Modifier modifier(fg, bg, fmt);
  std::stringstream ss;
  ss << modifier;
  EXPECT_EQ(modifier_to_string(fg, bg, fmt), ss.str());
}