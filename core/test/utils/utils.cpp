// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "utest/utils.hpp"

#include "core/utils/formatter.hpp"

using core::utils::TimeLabelFormatter;
using core::utils::TimeLabelModifierFormatter;

testing::AssertionResult operator&&(const testing::AssertionResult& lhs,
                                    const testing::AssertionResult& rhs) {
  return lhs ? rhs : lhs;
}

testing::AssertionResult AssertEqWithLabel(int expect, int actual,
                                           const char* label) {
  if (expect == actual) {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure()
         << label << " values are not equal.\n\tExpected: " << expect
         << "\n\tActual: " << actual << std::endl;
}

std::string ModifierToString(const std::vector<int>& options) {
  std::string str;
  for (const auto& opt : options) {
    str += "\x1B[" + std::to_string(opt) + "m";
  }
  return str;
}

void ExpectEqModifier(const std::vector<int>& expect_options,
                      const Modifier& actual) {
  EXPECT_EQ(ModifierToString(expect_options), actual.ToString());
}

void ExpectEqModifier(const Modifier& expect, const Modifier& actual) {
  EXPECT_EQ(expect.ToString(), actual.ToString());
}

std::string StreamExpectedLabeledModifier(std::string_view label,
                                          const Modifier& modifier) {
  std::stringstream ss;
  ss << modifier << label << core::utils::DefaultModifier();
  return ss.str();
}

void ExpectEqLabeledModifier(EventLevel expect_event,
                             const std::string& expect_label,
                             const Modifier& expect_modifier,
                             const LabeledModifier& actual) {
  EXPECT_EQ(expect_event, actual.GetEventLevel());
  EXPECT_EQ(expect_label, actual.GetLabel());
  ExpectEqModifier(expect_modifier, actual.GetModifier());
  std::stringstream ss;
  ss << expect_modifier << expect_label << core::utils::DefaultModifier();
  EXPECT_EQ(ss.str(), actual.ToString());
}

void ExpectEqLabeledModifier(const LabeledModifier& expect,
                             const LabeledModifier& actual) {
  ExpectEqLabeledModifier(expect.GetEventLevel(), expect.GetLabel(),
                          expect.GetModifier(), actual);
}

std::list<std::string> ReadAllLinesFromFile(std::string_view file_name) {
  std::string line;
  std::list<std::string> lines;
  std::ifstream file;
  file.open(file_name.data(), std::ios::in);
  if (file.is_open()) {
    while (std::getline(file, line)) {
      lines.push_back(line);
    }
    file.close();
  }
  return lines;
}

testing::AssertionResult AssertStringList(
  const std::list<std::string>& expect, const std::list<std::string>& actual) {
  if (expect.size() != actual.size()) {
    return testing::AssertionFailure() << "Size mismatch";
  }
  std::stringstream ss;
  const bool result = std::equal(
    expect.begin(), expect.end(), actual.begin(), [&ss](auto p1, auto p2) {
      if (p1.compare(p2) == 0) {
        return true;
      }
      ss << "\n expect: \"" << p1 << "\" and got: \"" << p2 << "\"";
      return false;
    });
  // const bool result = std::equal(expect.begin(), expect.end(),
  // actual.begin());
  {}
  if (result) {
    return testing::AssertionSuccess();
  }
  // auto first1 = expect.begin();
  // auto last1 = expect.end();
  // auto first2 = actual.begin();
  // for (; first1 != last1; ++first1, ++first2) {
  //   ss << "\n expect: " << *first1->c_str() << "and got: " <<
  //   first2->c_str();
  // }
  return testing::AssertionFailure() << "Data mismatch: " << ss.str();
}
