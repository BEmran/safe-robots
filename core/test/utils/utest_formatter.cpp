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

constexpr const char* kMsg = "message";

// Mock Null formatter function
std::string MockNullFormatter(const LabeledModifier& /*lm*/,
                              std::string_view msg) {
  return msg.data();
}

// Mock Time Label formatter function
std::string MockTimeLabelFormatter(const LabeledModifier& lm,
                                   std::string_view msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]"
     << "[" << lm.GetLabel() << "]"
     << " " << msg;
  return ss.str();
}

// Mock Time Label Modifier formatter function
std::string MockTimeLabelModifierFormatter(const LabeledModifier& lm,
                                           std::string_view msg) {
  std::stringstream ss;
  ss << "[" << DateTime().TimeToString() << "]" << lm << " " << msg;
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

// run different Events against the passed function and formatter
testing::AssertionResult AssertFormatter(const FormatFunc& expect_func,
                                         const Formatter& actual_formater) {
  const auto actual_func = [&actual_formater](auto lm, auto msg) {
    return actual_formater.Format(lm, msg);
  };
  return AssertFormatter(expect_func, actual_func);
}

// test Null formatter
TEST(NullFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockNullFormatter, NullFormatter));
}

// test Create Null formatter
TEST(NullFormatter, CreateFormatter) {
  const auto formatter = CreateNullFormatter();
  EXPECT_TRUE(AssertFormatter(MockNullFormatter, formatter));
}

// test Time Label formatter
TEST(TimeLabelFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockTimeLabelFormatter, TimeLabelFormatter));
}

// test Create Time Label formatter
TEST(TimeLabelFormatter, CreateFormatter) {
  const auto formatter = CreateTimeLabelFormatter();
  EXPECT_TRUE(AssertFormatter(MockTimeLabelFormatter, formatter));
}

// test Time Label Modifier formatter
TEST(TimeLabelModifierFormatter, Format) {
  EXPECT_TRUE(AssertFormatter(MockTimeLabelModifierFormatter,
                              TimeLabelModifierFormatter));
}

// test Create Time Label Modifier formatter
TEST(TimeLabelModifierFormatter, CreateFormatter) {
  const auto formatter = CreateTimeLabelModifierFormatter();
  EXPECT_TRUE(AssertFormatter(MockTimeLabelModifierFormatter, formatter));
}

// test Formatter with default Format function
TEST(Formatter, DefaultFormat) {
  const Formatter formatter;
  EXPECT_TRUE(AssertFormatter(NullFormatter, formatter));
}

// test Formatter with lambda callback function
TEST(Formatter, MockFormatter) {
  auto mock_format_func = [](const LabeledModifier& lm, std::string_view msg) {
    return lm.GetLabel() + ": " + msg.data();
  };
  const Formatter formatter(mock_format_func);
  EXPECT_TRUE(AssertFormatter(mock_format_func, formatter));
}
