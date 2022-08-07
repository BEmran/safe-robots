#include <gtest/gtest.h>

#include "core/utils/event_level.hpp"
#include "utils.hpp"

using namespace core::utils;
using event_level_t = EventLevel::event_level_t;

// test converting an event level to string
TEST(EventLevelToString, AllLevels)
{
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    EXPECT_EQ(LABELS[i], EventLevelToString(EVENTS[i]));
  }
}

// test constracting LabeledModifier with only event
TEST(LabeledModifier, ConstructWithEvent)
{
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    const LabeledModifier labeled(EVENTS[i]);
    expect_eq_labeled_modifier(EVENTS[i], LABELS[i], DefaultModifier(),
                               labeled);
  }
}

// test constracting LabeledModifier with different events and a label
TEST(LabeledModifier, ConstructWithEventAndLabel)
{
  const char* label = "label";
  for (const auto& event : EVENTS)
  {
    const LabeledModifier labeled(event, label);
    expect_eq_labeled_modifier(event, label, DefaultModifier(), labeled);
  }
}

// test constracting LabeledModifier with different events and a modifier
TEST(LabeledModifier, ConstructWithEventAndModifier)
{
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    const LabeledModifier labeled(EVENTS[i], modifier);
    expect_eq_labeled_modifier(EVENTS[i], LABELS[i], modifier, labeled);
  }
}

// test constracting LabeledModifier with different events, a label and a
// modifier
TEST(LabeledModifier, ConstructWithEventAndLabelAndModifier)
{
  Modifier modifier(FG::FG_LIGHT_BLUE, BG::BG_LIGHT_CYAN, FMT::FMT_HIDDEN);
  const char* label = "label";
  for (const auto& event : EVENTS)
  {
    const LabeledModifier labeled(event, label, modifier);
    expect_eq_labeled_modifier(event, label, modifier, labeled);
  }
}

// check debug labled modifier settings created using DebugLabeledModifier
// function
TEST(LabeledModifier, DebugLabeledModifier)
{
  const auto actual = DebugLabeledModifier();
  const auto event = EventLevel::EL_DEBUG;
  expect_eq_labeled_modifier(event, EventLevelToString(event), DebugModifier(),
                             actual);
}

// check error labled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, error_labeled_modifier)
{
  const auto actual = ErrorLabeledModifier();
  const auto event = EventLevel::EL_ERROR;
  expect_eq_labeled_modifier(event, EventLevelToString(event), ErrorModifier(),
                             actual);
}

// check info labled modifier settings created using info_labeled_modifier
// function
TEST(LabeledModifier, info_labeled_modifier)
{
  const auto actual = InfoLabeledModifier();
  const auto event = EventLevel::EL_INFO;
  expect_eq_labeled_modifier(event, EventLevelToString(event), InfoModifier(),
                             actual);
}

// check warn labled modifier settings created using warn_labeled_modifier
// function
TEST(LabeledModifier, warn_labeled_modifier)
{
  const auto actual = WarnLabeledModifier();
  const auto event = EventLevel::EL_WARN;
  expect_eq_labeled_modifier(event, EventLevelToString(event), WarnModifier(),
                             actual);
}