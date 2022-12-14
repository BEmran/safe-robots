// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "utest/utils.hpp"

#include "core/utils/formatter.hpp"

using namespace std::literals;

using core::utils::CreateTimeLabelFormatter;
using core::utils::CreateTimeLabelModifierFormatter;
using core::utils::Formatter;
using core::utils::TimeLabelFormatter;
using core::utils::TimeLabelModifierFormatter;

std::string ModifierToString(const std::vector<int>& options) {
  std::string str;
  for (const auto& opt : options) {
    str += "\x1B[" + std::to_string(opt) + "m";
  }
  return str;
}

testing::AssertionResult ExpectEqModifier(
  const std::vector<int>& expect_options, const Modifier& actual) {
  const auto expect = ModifierToString(expect_options);
  if (ModifierToString(expect_options) == actual.ToString()) {
    return testing::AssertionSuccess();
  } else {
    return AssertionFailureMsg(expect, actual.ToString());
  }
}

testing::AssertionResult ExpectEqModifier(const Modifier& expect,
                                          const Modifier& actual) {
  if (expect.ToString() == actual.ToString()) {
    return testing::AssertionSuccess();
  } else {
    return AssertionFailureMsg(expect, actual);
  }
}

std::string StreamExpectedLabeledModifier(const Modifier& modifier,
                                          std::string_view label) {
  std::stringstream ss;
  ss << modifier << label << core::utils::DefaultModifier();
  return ss.str();
}

testing::AssertionResult ExpectEqLabeledModifier(
  const EventLevel expect_event, const std::string& expect_label,
  const Modifier& expect_modifier, const LabeledModifier& actual) {
  if (expect_event != actual.GetEventLevel()) {
    return AssertionFailureMsg(expect_event, actual.GetEventLevel());
  }

  if (expect_label != actual.GetLabel()) {
    return AssertionFailureMsg(expect_label, actual.GetLabel());
  }

  if (expect_label != actual.GetLabel()) {
    return AssertionFailureMsg(expect_label, actual.GetLabel());
  }

  auto result = ExpectEqModifier(expect_modifier, actual.GetModifier());
  if (!result) {
    return result;
  }

  auto expect_stream =
    StreamExpectedLabeledModifier(expect_modifier, expect_label);
  if (expect_stream != actual.ToString()) {
    return AssertionFailureMsg(expect_stream, actual.ToString());
  }

  return testing::AssertionSuccess();
}

testing::AssertionResult ExpectEqLabeledModifier(
  const LabeledModifier& expect, const LabeledModifier& actual) {
  return ExpectEqLabeledModifier(expect.GetEventLevel(), expect.GetLabel(),
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
      // if not the end of file(more lines to read) append "\n" at the end
      if (not file.eof()) {
        lines.back() += "\n";
      }
    }
    file.close();
  }
  return lines;
}

testing::AssertionResult AssertStringList(
  const std::list<std::string>& expect, const std::list<std::string>& actual) {
  if (expect.size() != actual.size()) {
    return AssertionFailureMsg(expect.size(), actual.size(), "Size mismatch: ");
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
  if (result) {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure() << "Data mismatch: " << ss.str();
}

std::string ExpectLoggerMsg(std::string_view logger_name, std::string_view msg,
                            std::string_view end_msg) {
  std::string labeled_msg;
  if (logger_name.size() > 0) {
    labeled_msg += "["s + logger_name.data() + "] "s;
  }
  labeled_msg += msg.data();
  labeled_msg += end_msg.data();
  return labeled_msg;
}

std::string ExpectFormattedLoggerMsg(Formatter* formatter,
                                     const LabeledModifier lm,
                                     std::string_view logger_name,
                                     std::string_view msg,
                                     std::string_view end_msg) {
  const auto expected_logger_msg = ExpectLoggerMsg(logger_name, msg, end_msg);
  return formatter->Format(lm, expected_logger_msg);
}

std::string ExpectMsgForFileLogger(const LabeledModifier lm,
                                   std::string_view logger_name,
                                   std::string_view msg) {
  auto formatter = CreateTimeLabelFormatter();
  return ExpectFormattedLoggerMsg(&formatter, lm, logger_name, msg, "\n");
}

std::string ExpectMsgForStreamLogger(const LabeledModifier lm,
                                     std::string_view logger_name,
                                     std::string_view msg) {
  auto formatter = CreateTimeLabelModifierFormatter();
  return ExpectFormattedLoggerMsg(&formatter, lm, logger_name, msg, "\n");
}