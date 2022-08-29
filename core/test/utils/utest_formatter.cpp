// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/formatter.hpp"
#include "utest/utils.hpp"

using core::utils::DateTime;
using core::utils::DefaultFormater;
using core::utils::FormatterInterface;
using core::utils::NullFormatter;

// format a message with disabled Modifier
std::string FormatFuncWithModifierDisabled(const LabeledModifier& lm,
                                           const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]";
  ss << "[" << EventLevelToString(lm.GetEventLevel()) << "]";
  ss << ": " << msg;
  return ss.str();
}

// format a message with enabled Modifier
std::string FormatFuncWithModifierEnabled(const LabeledModifier& lm,
                                          const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]";
  ss << lm;
  ss << ": " << msg;
  return ss.str();
}

// test Null formatter
TEST(NullFormatter, Format) {
  std::string msg = "message";
  FormatterInterface* formatter = new NullFormatter();
  for (const auto& event : kEvents) {
    EXPECT_EQ(msg, formatter->Format(LabeledModifier(event), msg));
  }
}

// test Default formatter's Format function constructed without enabling the
// use of using modifier
TEST(DefaultFormater, FormatWithoutModifier) {
  std::string msg = "message";
  DefaultFormater formatter(false);
  for (size_t i = 0; i < kEvents.size(); i++) {
    LabeledModifier lm(kEvents[i], kLabels[i]);
    std::string expect = FormatFuncWithModifierDisabled(lm, msg);
    EXPECT_EQ(expect, formatter.Format(lm, msg));
  }
}

// test Default formatter's Format function constructed with enabling the use of
// modifier and time zone information
TEST(DefaultFormater, FormatWithModifier) {
  std::string msg = "message";
  DefaultFormater formatter(true);
  for (size_t i = 0; i < kEvents.size(); i++) {
    LabeledModifier lm(kEvents[i], kLabels[i]);
    std::string expect = FormatFuncWithModifierEnabled(lm, msg);
    EXPECT_EQ(expect, formatter.Format(lm, msg));
  }
}
