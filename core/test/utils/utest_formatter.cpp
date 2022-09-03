// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/formatter.hpp"
#include "utest/utils.hpp"

using core::utils::CreateNullFormatter;
using core::utils::CreateTimeLabelFormatter;
using core::utils::CreateTimeLabelModifierFormatter;
using core::utils::DateTime;
using core::utils::FormatFunc;
using core::utils::Formatter;
using core::utils::NullFormatter;
using core::utils::TimeLabelFormatter;
using core::utils::TimeLabelModifierFormatter;
using std::placeholders::_1;
using std::placeholders::_2;

constexpr const char* kMsg = "message";

// Mock Null formatter function
std::string MockNullFormatter(const LabeledModifier& /*lm*/,
                              const std::string& msg) {
  return msg;
}

// Mock Time Label formatter function
std::string MockTimeLabelFormatter(const LabeledModifier& lm,
                                   const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]"
     << "[" << lm.GetLabel() << "]"
     << ": " << msg;
  return ss.str();
}

// Mock Time Label Modifier formatter function
std::string MockTimeLabelModifierFormatter(const LabeledModifier& lm,
                                           const std::string& msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]" << lm << ": " << msg;
  return ss.str();
}

// run different Events against the passed functions
testing::AssertionResult AssertFormatter(const FormatFunc& expect_func,
                                         const FormatFunc& actual_func) {
  for (const auto& event : kEvents) {
    const auto lm = LabeledModifier(event);
    const auto expect = expect_func(lm, kMsg);
    const auto actual = actual_func(lm, kMsg);
    if (expect != actual) {
      return testing::AssertionFailure();
    }
  }
  return testing::AssertionSuccess();
}

// test Null formatter
TEST(NullFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockNullFormatter, NullFormatter));
}

// test Create Null formatter
TEST(NullFormatter, CreateFormatter) {
  const auto formatter = CreateNullFormatter();
  const auto actual_func = std::bind(&Formatter::Format, formatter, _1, _2);
  EXPECT_TRUE(AssertFormatter(MockNullFormatter, actual_func));
}

// test Time Label formatter
TEST(TimeLabelFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockTimeLabelFormatter, TimeLabelFormatter));
}

// test Create Time Label formatter
TEST(TimeLabelFormatter, CreateFormatter) {
  const auto formatter = CreateTimeLabelFormatter();
  const auto actual_func = std::bind(&Formatter::Format, formatter, _1, _2);
  EXPECT_TRUE(AssertFormatter(MockTimeLabelFormatter, actual_func));
}

// test Time Label Modifier formatter
TEST(TimeLabelModifierFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockTimeLabelModifierFormatter,
                              TimeLabelModifierFormatter));
}

// test Create Time Label Modifier formatter
TEST(TimeLabelModifierFormatter, CreateFormatter) {
  const auto formatter = CreateTimeLabelModifierFormatter();
  const auto actual_func = std::bind(&Formatter::Format, formatter, _1, _2);
  EXPECT_TRUE(AssertFormatter(MockTimeLabelModifierFormatter, actual_func));
}

// test Formatter with default Format function
TEST(Formatter, DefaultFormat) {
  const Formatter formatter;
  const auto actual_func = std::bind(&Formatter::Format, formatter, _1, _2);
  EXPECT_TRUE(AssertFormatter(NullFormatter, actual_func));
}

// test Formatter with lambda callback function
TEST(Formatter, MockFormatter) {
  auto mock_format_func = [](const LabeledModifier& lm,
                             const std::string& msg) -> std::string {
    return lm.GetLabel() + ": " + msg;
  };
  const auto formatter = std::make_shared<Formatter>(mock_format_func);
  const auto actual_func = std::bind(&Formatter::Format, formatter, _1, _2);
  EXPECT_TRUE(AssertFormatter(mock_format_func, actual_func));
}
