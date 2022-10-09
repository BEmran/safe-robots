// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/labeld_modifier.hpp"
#include "utest/utils.hpp"

using core::utils::DebugLabeledModifier;
using core::utils::DebugModifier;
using core::utils::DefaultModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::ErrorModifier;
using core::utils::InfoLabeledModifier;
using core::utils::InfoModifier;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

// test converting an event level to string
TEST(EventLevelToString, AllLevels) {
  for (size_t i = 0; i < kEvents.size(); i++) {
    EXPECT_EQ(kLabels[i], EventLevelToString(kEvents[i]));
  }
}

// test constructing LabeledModifier with only event, should have
// DefaultModifier as modifier and event's string as label
TEST(LabeledModifier, ConstructWithEvent) {
  for (size_t i = 0; i < kEvents.size(); i++) {
    const LabeledModifier lm(kEvents[i]);
    ExpectEqLabeledModifier(kEvents[i], EventLevelToString(kEvents[i]),
                            DefaultModifier(), lm);
  }
}

// test constructing LabeledModifier with event and a label, should have
// DefaultModifier as modifier
TEST(LabeledModifier, ConstructWithEventAndLabel) {
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label);
    ExpectEqLabeledModifier(event, label, DefaultModifier(), lm);
  }
}

// test constructing LabeledModifier with different events and a modifier,
// should have event's string as label
TEST(LabeledModifier, ConstructWithEventAndModifier) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  for (size_t i = 0; i < kEvents.size(); i++) {
    const LabeledModifier lm(kEvents[i], modifier);
    ExpectEqLabeledModifier(kEvents[i], EventLevelToString(kEvents[i]),
                            modifier, lm);
  }
}

// test constructing LabeledModifier with different events, a label and a
// modifier
TEST(LabeledModifier, ConstructWithEventAndLabelAndModifier) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label, modifier);
    ExpectEqLabeledModifier(event, label, modifier, lm);
  }
}

// test streaming LabeledModifier for different events
TEST(LabeledModifier, TestStream) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  const char* label = "label";
  for (const auto& event : kEvents) {
    const LabeledModifier lm(event, label, modifier);
    std::stringstream ss;
    ss << lm;
    EXPECT_EQ(StreamExpectedLabeledModifier(label, modifier), ss.str());
  }
}

// test streaming LabeledModifier for different events
TEST(LabeledModifier, TestToString) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
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
TEST(LabeledModifier, DebugLabeledModifier) {
  const auto actual = DebugLabeledModifier();
  const auto event = EventLevel::DEBUG;
  ExpectEqLabeledModifier(event, EventLevelToString(event), DebugModifier(),
                          actual);
}

// check error labeled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, error_labeled_modifier) {
  const auto actual = ErrorLabeledModifier();
  const auto event = EventLevel::ERROR;
  ExpectEqLabeledModifier(event, EventLevelToString(event), ErrorModifier(),
                          actual);
}

// check info labeled modifier settings created using info_labeled_modifier
// function
TEST(LabeledModifier, info_labeled_modifier) {
  const auto actual = InfoLabeledModifier();
  const auto event = EventLevel::INFO;
  ExpectEqLabeledModifier(event, EventLevelToString(event), InfoModifier(),
                          actual);
}

// check warn labeled modifier settings created using warn_labeled_modifier
// function
TEST(LabeledModifier, warn_labeled_modifier) {
  const auto actual = WarnLabeledModifier();
  const auto event = EventLevel::WARN;
  ExpectEqLabeledModifier(event, EventLevelToString(event), WarnModifier(),
                          actual);
}
