// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <tuple>
#include <utility>

#include "core/utils/formatter2.hpp"
#include "core/utils/modifier.hpp"
#include "utest/utils.hpp"

using core::utils::DateTime;
using core::utils::FormatterInterface;
using core::utils::LabeledModifier;
using core::utils::NullFormatter;
using core::utils::TimeFormatter;
using core::utils::TupleFormatter;
using core::utils::TupleToString;

constexpr const char* kMsg = "message";

/**
 * @brief template function to help getting get the size of passed tuple and
 * then pass it again to TupleToString
 * @param t tuple to be printed
 * @return std::string return of TupleToString
 */
template <class... Args>
std::string TupleToStringHelper(const std::tuple<Args...>& t) {
  return TupleToString(t, std::index_sequence_for<Args...>{});
}

testing::AssertionResult AssertFormatter(const std::string& expect_header,
                                         FormatterInterface* formatter) {
  auto expect = expect_header + std::string(kMsg);
  if (expect == formatter->Format(kMsg)) {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure();
}

// test Tuple To String
TEST(TupleToString, Test) {
  std::tuple<int, double> tuple(1, 2.3);
  std::string expect = "[1][2.3]";
  EXPECT_EQ(expect, TupleToStringHelper(tuple));
}

// test Null formatter
TEST(NullFormatter, Format) {
  EXPECT_EQ(kMsg, NullFormatter().Format(kMsg));
}

// test formatter
TEST(Formatter, FormatOneType) {
  TupleFormatter<std::string_view> f("Node");
  const std::string expect_header = "[Node] ";
  EXPECT_TRUE(AssertFormatter(expect_header, &f));
}

TEST(Formatter, FormatOneRefType) {
  std::string name = "node 1";
  TupleFormatter<std::string&> f(name);
  const std::string expect_header1 = "[" + name + "] ";
  EXPECT_TRUE(AssertFormatter(expect_header1, &f));
  name = "node 2";
  const std::string expect_header2 = "[" + name + "] ";
  EXPECT_TRUE(AssertFormatter(expect_header2, &f));
}

TEST(Formatter, FormatTwoType) {
  TupleFormatter<std::string_view, int> f("Node", 123);
  const std::string expect_header = "[Node][123] ";
  EXPECT_TRUE(AssertFormatter(expect_header, &f));
}

TEST(Formatter, FormatWithModifier) {
  Modifier mod = core::utils::DefaultModifier();
  TupleFormatter<Modifier> f(mod);
  const std::string expect_header = "[" + mod.ToString() + "] ";
  EXPECT_TRUE(AssertFormatter(expect_header, &f));
}

TEST(Formatter, FormatWithLabeledModifier) {
  LabeledModifier lm = core::utils::DebugLabeledModifier();
  TupleFormatter<LabeledModifier> f(lm);
  const std::string expect_header = "[" + lm.ToString() + "] ";
  EXPECT_TRUE(AssertFormatter(expect_header, &f));
}

// test Time Formatter
TEST(TimeFormatter, FormatOneType) {
  TimeFormatter<std::string_view> f("Node");
  const std::string expect_header =
    "[" + DateTime().TimeToString() + "][Node] ";
  EXPECT_TRUE(AssertFormatter(expect_header, &f));
}