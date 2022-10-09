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

std::string StreamExpectedLabeledModifier(std::string_view label,
                                          const Modifier& modifier) {
  std::stringstream ss;
  ss << modifier << label << core::utils::DefaultModifier();
  return ss.str();
}

std::string ModifierToString(FG fg, BG bg, FMT fmt) {
  constexpr auto kBuffSize = 25;
  std::string buffer(kBuffSize, ' ', std::allocator<char>());
  const auto actual_size = snprintf(buffer.data(), buffer.size(),
                                    "\x1B[%dm\x1B[%dm\x1B[%dm", fmt, fg, bg);
  buffer.resize(static_cast<size_t>(actual_size));
  return buffer;
}

void ExpectEqModifier(FG expect_fg, BG expect_bg, FMT expect_fmt,
                      const Modifier& actual) {
  EXPECT_EQ(ModifierToString(expect_fg, expect_bg, expect_fmt),
            actual.ToString());
}

void ExpectEqModifier(const Modifier& expect, const Modifier& actual) {
  EXPECT_EQ(expect.ToString(), actual.ToString());
}

void ExpectEqLabeledModifier(EventLevel expect_event,
                             const std::string& expect_label,
                             const Modifier& expect_modifier,
                             const LabeledModifier& actual) {
  EXPECT_EQ(expect_event, actual.GetEventLevel());
  EXPECT_EQ(expect_label, actual.GetLabel());
  ExpectEqModifier(expect_modifier, actual.GetModifier());
}

void ExpectEqLabeledModifier(const LabeledModifier& expect,
                             const LabeledModifier& actual) {
  ExpectEqLabeledModifier(expect.GetEventLevel(), expect.GetLabel(),
                          expect.GetModifier(), actual);
}

std::list<std::string> ReadAllLinesFromFile(const std::string& file_name) {
  std::string line;
  std::list<std::string> lines;
  std::ifstream file;
  file.open(file_name, std::ios::in);
  if (file.is_open()) {
    while (std::getline(file, line)) {
      lines.push_back(line);
    }
    file.close();
  }
  return lines;
}

ConsoleBuffer::ConsoleBuffer(const std::string& file_name)
  : file_name_(file_name) {
  file_.open(file_name_, std::ios_base::out);
  backup = std::cout.rdbuf();  // back up cout's streambuf
  file_buf_ = file_.rdbuf();   // get file's streambuf
  std::cout.rdbuf(file_buf_);  // assign streambuf to cout
}

ConsoleBuffer::~ConsoleBuffer() {
  if (!file_.is_open()) {
    file_.close();
  }
}

std::list<std::string> ConsoleBuffer::RestoreCoutBuffer() const {
  // restore cout's original buffer
  std::cout.rdbuf(backup);
  return ReadAllLinesFromFile(file_name_);
}

testing::AssertionResult AssertStringList(
  const std::list<std::string>& expect, const std::list<std::string>& actual) {
  if (expect.size() != actual.size()) {
    return testing::AssertionFailure() << "Size mismatch";
  }

  if (!std::equal(expect.begin(), expect.end(), actual.begin())) {
    return testing::AssertionFailure() << "Data mismatch";
  }

  return testing::AssertionSuccess();
}

testing::AssertionResult AssertFileAndConsole(const std::string& filename,
                                              const ConsoleBuffer& c_buffer,
                                              const LabeledModifier& lm,
                                              const std::string& msg) {
  const auto f_logged_data = ReadAllLinesFromFile(filename);
  const auto f_expect = TimeLabelFormatter(lm, msg);
  const auto res1 = AssertStringList({f_expect}, f_logged_data);

  const auto c_logged_data = c_buffer.RestoreCoutBuffer();
  const auto c_expect = TimeLabelModifierFormatter(lm, msg);
  const auto res2 = AssertStringList({c_expect}, c_logged_data);

  return res1 && res2;
}
