#ifndef TEST_UTEST_UTILS_HPP
#define TEST_UTEST_UTILS_HPP

#include <array>
#include <core/utils/event_level.hpp>
#include <core/utils/terminal.hpp>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <list>
#include <string>

#include "gtest/gtest.h"

typedef core::utils::Modifier Modifier;
typedef core::utils::LabeledModifier LabeledModifier;
typedef core::utils::EventLevel EventLevel;
typedef core::utils::terminal::FG FG;
typedef core::utils::terminal::BG BG;
typedef core::utils::terminal::FMT FMT;

// careate list of available variables
const char* LABELS[] = {"INFO", "DEBUG", "WARN", "ERROR"};
const std::vector<EventLevel::event_level_t> EVENTS = {
    EventLevel::EL_INFO, EventLevel::EL_DEBUG, EventLevel::EL_WARN,
    EventLevel::EL_ERROR};

/**
 * @brief applu and operation of two AssertionResult
 *
 * @param lhs AssertionResult of the left hand side
 * @param rhs AssertionResult of the right hand side
 * @return testing::AssertionResult the and result
 */
testing::AssertionResult operator&&(const testing::AssertionResult& lhs,
                                    const testing::AssertionResult& rhs)
{
  return lhs == true ? rhs : lhs;
}

/**
 * @brief assert if two values are equal with additional information
 *
 * @tparam T variables type
 * @param expect expected value
 * @param actual actual value
 * @param label values label
 * @return testing::AssertionResult assertation result
 */
template <typename T>
testing::AssertionResult AssertEqWithLabel(const T expect, const T actual,
                                           const char* label)
{
  if (expect == actual)
  {
    return testing::AssertionSuccess();
  }
  return testing::AssertionFailure()
         << label << " values are not equal.\n\tExpected: " << expect
         << "\n\tActual: " << actual << std::endl;
}

// generate a string using the passed modifier and its label
std::string FormatLabeledModifier(const Modifier& modifier,
                                  const std::string& label)
{
  std::stringstream ss;
  ss << modifier << "[" << label << "]" << core::utils::DefaultModifier();
  return ss.str();
}

// convert Modifier to a string
std::string modifier_to_string(const FG fg, const BG bg, const FMT fmt)
{
  char buffer[25];
  sprintf(buffer, "\x1B[%dm\x1B[%dm\x1B[%dm", fmt, fg, bg);
  return buffer;
}

// check if the passed modifier has the passed configuration
void ExpectEqModifier(const FG expect_fg, const BG expect_bg,
                      const FMT expect_fmt, const Modifier& actual)
{
  EXPECT_EQ(modifier_to_string(expect_fg, expect_bg, expect_fmt),
            actual.ToString());
}

// check if the passed modifiers have the same configuration
void ExpectEqModifier(const Modifier& expect, const Modifier& actual)
{
  EXPECT_EQ(expect.ToString(), actual.ToString());
}

// check if the passed labeled-modifier has the same expected configuration
void ExpectEqLabeledModifier(const EventLevel::event_level_t expect_event,
                             const std::string& expect_label,
                             const Modifier& expect_modifier,
                             const LabeledModifier& actual)
{
  EXPECT_EQ(expect_event, actual.GetEventLevel());
  EXPECT_EQ(expect_label, actual.GetLabel());
  ExpectEqModifier(expect_modifier, actual.GetModifier());
  std::stringstream ss;
  ss << actual;
  EXPECT_EQ(FormatLabeledModifier(expect_modifier, expect_label), ss.str());
}

// check if the passed labeled-modifiers have the expected configuration
void ExpectEqLabeledModifier(const LabeledModifier& expect,
                             const LabeledModifier& actual)
{
  ExpectEqLabeledModifier(expect.GetEventLevel(), expect.GetLabel(),
                          expect.GetModifier(), actual);
}

/**
 * @brief Reads all lines from a file
 *
 * @param file_name file name to read from
 * @return std::list<std::string> data written in a file
 */
std::list<std::string> ReadAllLinesFromFile(const std::string& file_name)
{
  std::string line;
  std::list<std::string> lines;
  std::ifstream file;
  file.open(file_name, std::ios::in);
  if (file.is_open())
  {
    while (std::getline(file, line))
    {
      lines.push_back(line);
    }
    file.close();
  }
  return lines;
}

/**
 * @brief This allows to trace what is printout in the stream and write it in
 * a file instead in order to be checked latter
 *
 */
class ConsoleBuffer
{
 public:
  ConsoleBuffer(const std::string& file_name = "console.txt")
    : file_name_(file_name)
  {
    file_.open(file_name_, std::ios_base::out);
    backup = std::cout.rdbuf();  // back up cout's streambuf
    psbuf = file_.rdbuf();       // get file's streambuf
    std::cout.rdbuf(psbuf);      // assign streambuf to cout
  }

  ~ConsoleBuffer()
  {
    if (!file_.is_open())
    {
      file_.close();
    }
  }

  /**
   * @brief Restores the string printed at cout
   *
   * @return std::list<std::string> lines printed to cout
   */
  std::list<std::string> RestoreCoutBuffer()
  {
    // restore cout's original buffer
    std::cout.rdbuf(backup);
    return ReadAllLinesFromFile(file_name_);
  }
  std::string file_name_;
  std::ofstream file_;
  std::streambuf *psbuf, *backup;
};

#endif  // TEST_UTEST_UTILS_HPP