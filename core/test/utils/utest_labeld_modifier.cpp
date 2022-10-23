// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/labeld_modifier.hpp"
#include "utest/utils.hpp"

namespace FG = core::utils::FG;
namespace BG = core::utils::BG;
namespace FMT = core::utils::FMT;
using core::utils::DebugLabeledModifier;
using core::utils::DebugModifier;
using core::utils::DefaultModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::ErrorModifier;
using core::utils::FatalLabeledModifier;
using core::utils::FatalModifier;
using core::utils::InfoLabeledModifier;
using core::utils::InfoModifier;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

// test constructing LabeledModifier with only event, should have
// DefaultModifier as modifier and event's string as label
TEST(LabeledModifier, ConstructWithEvent) {
  for (size_t i = 0; i < kEvents.size(); i++) {
    const LabeledModifier lm(kEvents[i]);
    EXPECT_TRUE(ExpectEqLabeledModifier(
      kEvents[i], EventLevelToString(kEvents[i]), Modifier(), lm));
  }
}

// test constructing LabeledModifier with event and a label, should have
// DefaultModifier as modifier
TEST(LabeledModifier, ConstructWithEventAndLabel) {
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label);
    EXPECT_TRUE(ExpectEqLabeledModifier(event, label, Modifier(), lm));
  }
}

// test constructing LabeledModifier with different events and a modifier,
// should have event's string as label
TEST(LabeledModifier, ConstructWithEventAndModifier) {
  Modifier modifier({FG::LIGHT_BLUE, BG::LIGHT_CYAN, FMT::HIDDEN});
  for (size_t i = 0; i < kEvents.size(); i++) {
    const LabeledModifier lm(kEvents[i], modifier);
    EXPECT_TRUE(ExpectEqLabeledModifier(
      kEvents[i], EventLevelToString(kEvents[i]), modifier, lm));
  }
}

// test constructing LabeledModifier with different events, a label and a
// modifier
TEST(LabeledModifier, ConstructWithEventAndLabelAndModifier) {
  Modifier modifier({FG::LIGHT_BLUE, BG::LIGHT_CYAN, FMT::HIDDEN});
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label, modifier);
    EXPECT_TRUE(ExpectEqLabeledModifier(event, label, modifier, lm));
  }
}

// test streaming LabeledModifier for different events
TEST(LabeledModifier, TestStream) {
  Modifier modifier({FG::LIGHT_BLUE, BG::LIGHT_CYAN, FMT::HIDDEN});
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label, modifier);
    std::stringstream ss;
    ss << lm;
    EXPECT_EQ(StreamExpectedLabeledModifier(modifier, label), ss.str());
  }
}

// test streaming LabeledModifier for different events
TEST(LabeledModifier, TestToString) {
  Modifier modifier({FG::LIGHT_BLUE, BG::LIGHT_CYAN, FMT::HIDDEN});
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label, modifier);
    std::string expect =
      modifier.ToString() + label + DefaultModifier().ToString();
    EXPECT_EQ(expect, lm.ToString());
  }
}

// check debug labeled modifier settings created using DebugLabeledModifier
// function
TEST(LabeledModifier, Debug) {
  const auto actual = DebugLabeledModifier();
  const auto event = EventLevel::DEBUG;
  EXPECT_TRUE(ExpectEqLabeledModifier(event, EventLevelToString(event),
                                      DebugModifier(), actual));
}

// check error labeled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, Error) {
  const auto actual = ErrorLabeledModifier();
  const auto event = EventLevel::ERROR;
  EXPECT_TRUE(ExpectEqLabeledModifier(event, EventLevelToString(event),
                                      ErrorModifier(), actual));
}

// check fatal labeled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, Fatal) {
  const auto actual = FatalLabeledModifier();
  const auto event = EventLevel::FATAL;
  EXPECT_TRUE(ExpectEqLabeledModifier(event, EventLevelToString(event),
                                      FatalModifier(), actual));
}

// check info labeled modifier settings created using info_labeled_modifier
// function
TEST(LabeledModifier, Info) {
  const auto actual = InfoLabeledModifier();
  const auto event = EventLevel::INFO;
  EXPECT_TRUE(ExpectEqLabeledModifier(event, EventLevelToString(event),
                                      InfoModifier(), actual));
}

// check warn labeled modifier settings created using warn_labeled_modifier
// function
TEST(LabeledModifier, Warn) {
  const auto actual = WarnLabeledModifier();
  const auto event = EventLevel::WARN;
  EXPECT_TRUE(ExpectEqLabeledModifier(event, EventLevelToString(event),
                                      WarnModifier(), actual));
}
