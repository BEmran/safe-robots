// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/modifier.hpp"
#include "utest/utils.hpp"

namespace FG = core::utils::FG;
namespace BG = core::utils::BG;
namespace FMT = core::utils::FMT;
using core::utils::DebugModifier;
using core::utils::DefaultModifier;
using core::utils::ErrorModifier;
using core::utils::FatalModifier;
using core::utils::InfoModifier;
using core::utils::WarnModifier;

// check a modifier settings created using its constructor
TEST(Modifier, EmptyConstruct) {
  std::vector<int> options;
  const Modifier modifier(options);
  ExpectEqModifier(options, modifier);
}

TEST(Modifier, SingleOptionConstruct) {
  std::vector<int> options{FG::CYAN};
  const Modifier modifier(options);
  ExpectEqModifier(options, modifier);
}

TEST(Modifier, DoubleOptionsConstruct) {
  std::vector<int> options{FG::CYAN, BG::GREEN};
  const Modifier modifier(options);
  ExpectEqModifier(options, modifier);
}

TEST(Modifier, TripleOptionsConstruct) {
  std::vector<int> options{FG::CYAN, BG::GREEN, FMT::DIM};
  const Modifier modifier(options);
  ExpectEqModifier(options, modifier);
}

// check default modifier settings created using DefaultModifier function
TEST(Modifier, DefaultModifier) {
  const Modifier modifier = DefaultModifier();
  std::vector<int> options{FG::DEFAULT, BG::DEFAULT, FMT::DEFAULT};
  ExpectEqModifier(options, modifier);
}

// check debug modifier settings created using DebugModifier function
TEST(Modifier, DebugModifier) {
  const Modifier modifier = DebugModifier();
  std::vector<int> options{FG::LIGHT_BLUE, BG::DEFAULT, FMT::BOLD};
  ExpectEqModifier(options, modifier);
}

// check error modifier settings created using ErrorModifier function
TEST(Modifier, ErrorModifier) {
  const Modifier modifier = ErrorModifier();
  std::vector<int> options{FG::LIGHT_RED, BG::DEFAULT, FMT::BOLD};
  ExpectEqModifier(options, modifier);
}

// check error modifier settings created using FatalModifier function
TEST(Modifier, FatalModifier) {
  const Modifier modifier = FatalModifier();
  std::vector<int> options{FG::RED, BG::YELLOW, FMT::BOLD, FMT::BLINK};
  ExpectEqModifier(options, modifier);
}

// check info modifier settings created using InfoModifier function
TEST(Modifier, InfoModifier) {
  const Modifier modifier = InfoModifier();
  std::vector<int> options{FG::LIGHT_CYAN, BG::DEFAULT, FMT::BOLD};
  ExpectEqModifier(options, modifier);
}

// check warning modifier settings created using WarnModifier function
TEST(Modifier, WarnModifier) {
  Modifier modifier = WarnModifier();
  std::vector<int> options{FG::LIGHT_YELLOW, BG::DEFAULT, FMT::BOLD};
  ExpectEqModifier(options, modifier);
}

// check overloading the << operator
TEST(Terminal, PrintOutModifier) {
  std::vector<int> options{FG::LIGHT_MAGENTA, BG::LIGHT_GRAY, FMT::BLINK,
                           FMT::UNDERLINE};
  const Modifier modifier(options);
  std::stringstream ss;
  ss << modifier;
  EXPECT_EQ(ModifierToString(options), ss.str());
}
