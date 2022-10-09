// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef UTEST_UTILS_HPP_
#define UTEST_UTILS_HPP_

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "core/utils/labeld_modifier.hpp"
#include "core/utils/terminal.hpp"
#include "gtest/gtest.h"

using core::utils::EventLevel;
using core::utils::LabeledModifier;
using core::utils::Modifier;
using core::utils::terminal::BG;
using core::utils::terminal::FG;
using core::utils::terminal::FMT;

// carate list of available variables
constexpr const char* kLabels[] = {"INFO", "DEBUG", "WARN", "ERROR"};
const std::vector<EventLevel> kEvents = {EventLevel::INFO, EventLevel::DEBUG,
                                         EventLevel::WARN, EventLevel::ERROR};

/**
 * @brief apply and operation of two AssertionResult
 *
 * @param lhs AssertionResult of the left hand side
 * @param rhs AssertionResult of the right hand side
 * @return testing::AssertionResult the and result
 */
testing::AssertionResult operator&&(const testing::AssertionResult& lhs,
                                    const testing::AssertionResult& rhs);

/**
 * @brief assert if two values are equal with additional information
 *
 * @param expect expected value
 * @param actual actual value
 * @param label values label
 * @return testing::AssertionResult assentation result
 */
testing::AssertionResult AssertEqWithLabel(int expect, int actual,
                                           const char* label);

// generate a string using the passed modifier and its label
std::string FormatLabeledModifier(const Modifier& modifier,
                                  const std::string& label);

// convert Modifier to a string
std::string ModifierToString(FG fg, BG bg, FMT fmt);

// check if the passed modifier has the passed configuration
void ExpectEqModifier(FG expect_fg, BG expect_bg, FMT expect_fmt,
                      const Modifier& actual);

// check if the passed modifiers have the same configuration
void ExpectEqModifier(const Modifier& expect, const Modifier& actual);

// check if the passed labeled-modifier has the same expected configuration
void ExpectEqLabeledModifier(EventLevel expect_event,
                             const std::string& expect_label,
                             const Modifier& expect_modifier,
                             const LabeledModifier& actual);

// check if the passed labeled-modifiers have the expected configuration
void ExpectEqLabeledModifier(const LabeledModifier& expect,
                             const LabeledModifier& actual);

/**
 * @brief Reads all lines from a file
 *
 * @param file_name file name to read from
 * @return std::list<std::string> data written in a file
 */
std::list<std::string> ReadAllLinesFromFile(const std::string& file_name);

/**
 * @brief This allows to trace what is printout in the stream and write it in
 * a file instead in order to be checked latter
 *
 */
class ConsoleBuffer {
 public:
  explicit ConsoleBuffer(const std::string& file_name = "console.txt");

  ~ConsoleBuffer();

  /**
   * @brief Restores the string printed at cout
   *
   * @return std::list<std::string> lines printed to cout
   */
  std::list<std::string> RestoreCoutBuffer() const;

  std::string file_name_;
  std::ofstream file_;
  std::streambuf *file_buf_, *backup;
};

testing::AssertionResult AssertStringVector(
  const std::list<std::string>& expect, const std::list<std::string>& actual);

testing::AssertionResult AssertFileAndConsole(const std::string& filename,
                                              const ConsoleBuffer& c_buffer,
                                              const LabeledModifier& lm,
                                              const std::string& msg);
#endif  // UTEST_UTILS_HPP_
