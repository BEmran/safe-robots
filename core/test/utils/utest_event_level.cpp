#include <gtest/gtest.h>

#include "core/utils/event_level.hpp"
#include "utest/utils.hpp"

using namespace core::utils;
using event_level_t = EventLevel::event_level_t;

// test converting an event level to string
TEST(EventLevelToString, AllLevels)
{
  for (size_t i = 0; i < EVENTS.size(); i++)
    EXPECT_EQ(LABELS[i], event_level_to_string(EVENTS[i]));
}

// test constracting LabeledModifier with only event
TEST(LabeledModifier, ConstructWithEvent)
{
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    const LabeledModifier labeled(EVENTS[i]);
    expect_eq_labeled_modifier(EVENTS[i], LABELS[i], default_modifier(),
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
    expect_eq_labeled_modifier(event, label, default_modifier(), labeled);
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
  const auto actual = debug_labeled_modifier();
  const auto event = event_level_t::EL_DEBUG;
  expect_eq_labeled_modifier(event, event_level_to_string(event),
                             debug_modifier(), actual);
}

// check error labled modifier settings created using error_labeled_modifier
// function
TEST(LabeledModifier, error_labeled_modifier)
{
  const auto actual = error_labeled_modifier();
  const auto event = event_level_t::EL_ERROR;
  expect_eq_labeled_modifier(event, event_level_to_string(event),
                             error_modifier(), actual);
}

// check info labled modifier settings created using info_labeled_modifier
// function
TEST(LabeledModifier, info_labeled_modifier)
{
  const auto actual = info_labeled_modifier();
  const auto event = event_level_t::EL_INFO;
  expect_eq_labeled_modifier(event, event_level_to_string(event),
                             info_modifier(), actual);
}

// check warn labled modifier settings created using warn_labeled_modifier
// function
TEST(LabeledModifier, warn_labeled_modifier)
{
  const auto actual = warn_labeled_modifier();
  const auto event = event_level_t::EL_WARN;
  expect_eq_labeled_modifier(event, event_level_to_string(event),
                             warn_modifier(), actual);
}