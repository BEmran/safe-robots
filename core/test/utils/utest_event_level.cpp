// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/event_level.hpp"
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
  for (size_t i = 0; i < EVENTS.size(); i++) {
    EXPECT_EQ(LABELS[i], EventLevelToString(EVENTS[i]));
  }
}

// test constructing LabeledModifier with only event
TEST(LabeledModifier, ConstructWithEvent) {
  for (size_t i = 0; i < EVENTS.size(); i++) {
    const LabeledModifier labeled(EVENTS[i]);
    ExpectEqLabeledModifier(EVENTS[i], LABELS[i], DefaultModifier(), labeled);
  }
}

// test constructing LabeledModifier with different events and a label
TEST(LabeledModifier, ConstructWithEventAndLabel) {
  const char* label = "label";
  for (const auto& event : EVENTS) {
    const LabeledModifier labeled(event, label);
    ExpectEqLabeledModifier(event, label, DefaultModifier(), labeled);
  }
}

// test constructing LabeledModifier with different events and a modifier
TEST(LabeledModifier, ConstructWithEventAndModifier) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  for (size_t i = 0; i < EVENTS.size(); i++) {
    const LabeledModifier labeled(EVENTS[i], modifier);
    ExpectEqLabeledModifier(EVENTS[i], LABELS[i], modifier, labeled);
  }
}

// test constructing LabeledModifier with different events, a label and a
// modifier
TEST(LabeledModifier, ConstructWithEventAndLabelAndModifier) {
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  const char* label = "label";
  for (const auto& event : EVENTS) {
    const LabeledModifier labeled(event, label, modifier);
    ExpectEqLabeledModifier(event, label, modifier, labeled);
  }
}

// check debug labeled modifier settings created using DebugLabeledModifier
// function
TEST(LabeledModifier, DebugLabeledModifier) {
  const auto actual = DebugLabeledModifier();
  const auto event = EventLevel::EL_DEBUG;
  ExpectEqLabeledModifier(event, EventLevelToString(event), DebugModifier(),
                          actual);
}

// check error labeled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, error_labeled_modifier) {
  const auto actual = ErrorLabeledModifier();
  const auto event = EventLevel::EL_ERROR;
  ExpectEqLabeledModifier(event, EventLevelToString(event), ErrorModifier(),
                          actual);
}

// check info labeled modifier settings created using info_labeled_modifier
// function
TEST(LabeledModifier, info_labeled_modifier) {
  const auto actual = InfoLabeledModifier();
  const auto event = EventLevel::EL_INFO;
  ExpectEqLabeledModifier(event, EventLevelToString(event), InfoModifier(),
                          actual);
}

// check warn labeled modifier settings created using warn_labeled_modifier
// function
TEST(LabeledModifier, warn_labeled_modifier) {
  const auto actual = WarnLabeledModifier();
  const auto event = EventLevel::EL_WARN;
  ExpectEqLabeledModifier(event, EventLevelToString(event), WarnModifier(),
                          actual);
}
