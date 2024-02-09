// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef UTEST_UTILS_HPP_
#define UTEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <list>
#include <string>
#include <string_view>
#include <vector>

#include "core/utils/formatter.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/modifier.hpp"

using core::utils::EventLevel;
using core::utils::Formatter;
using core::utils::LabeledModifier;
using core::utils::Modifier;
namespace BG = core::utils::BG;
namespace FG = core::utils::FG;
namespace FMT = core::utils::FMT;

/// @brief carate list of available variables
constexpr const char* kLabels[] = {"DEBUG", "ERROR", "FATAL", "INFO", "WARN"};

/// @brief should be same order as kLabels
const std::vector<EventLevel> kEvents = {EventLevel::DEBUG, EventLevel::ERROR,
                                         EventLevel::FATAL, EventLevel::INFO,
                                         EventLevel::WARN};
/// @brief epsilon used for comparison
constexpr auto kEpsilon = 0.00001;

template <typename T>
inline bool IsEqual(const T& actual, const T& expect) {
  return actual == expect;
}

template <>
inline bool IsEqual(const float& actual, const float& expect) {
  return std::fabs(actual - expect) < static_cast<float>(kEpsilon);
}

template <>
inline bool IsEqual(const double& actual, const double& expect) {
  return std::fabs(actual - expect) < static_cast<double>(kEpsilon);
}

/**
 * @brief Create failure assertion with information
 *
 * @param expect expected value
 * @param actual actual value
 * @param msg string to append at the beginning of failure msg
 * @return testing::AssertionResult failure assentation with msg
 */
template <typename T>
[[nodiscard]] ::testing::AssertionResult AssertionFailureMsg(
  const T& expect, const T& actual, std::string_view msg = "") {
  return testing::AssertionFailure()
         << msg << " expect: \'" << expect << "\' actual: \'" << actual << "\'";
}

/**
 * @brief assert if two values are equal with message to display at failure
 *
 * @param expect expected value
 * @param actual actual value
 * @param msg string to display at failure
 * @return testing::AssertionResult assentation result
 */
template <typename T>
[[nodiscard]] ::testing::AssertionResult ExpectEq(const T& expect,
                                                  const T& actual,
                                                  std::string_view msg = "") {
  if (IsEqual(actual, expect)) {
    return ::testing::AssertionSuccess();
  } else {
    return AssertionFailureMsg(expect, actual, msg);
  }
}

/**
 * @brief Apply && operation of two AssertionResults
 *
 * @param lhs AssertionResult of the left hand side
 * @param rhs AssertionResult of the right hand side
 * @return testing::AssertionResult the and result
 */
[[nodiscard]] inline testing::AssertionResult operator&&(
  const testing::AssertionResult& lhs, const testing::AssertionResult& rhs) {
  return lhs ? rhs : lhs;
}

/**
 * @brief Convert Modifier to a string
 *
 * @param options modifier options
 * @return std::string resulted conversion
 */
std::string ModifierToString(const std::vector<int>& options);

/**
 * @brief Check if the actual modifier has the expected options
 *
 * @param expect_options options of the expected modifier
 * @param actual actual modifier
 * @return testing::AssertionResult assertion result
 */
[[nodiscard]] testing::AssertionResult ExpectEqModifier(
  const std::vector<int>& expect_options, const Modifier& actual);

/**
 * @brief Check if the passed modifiers have the same configuration
 *
 * @param expect expect modifier
 * @param actual actual modifier
 * @return testing::AssertionResult assertion result
 */
[[nodiscard]] testing::AssertionResult ExpectEqModifier(const Modifier& expect,
                                                        const Modifier& actual);

/**
 * @brief Generate a string using stream method of the passed label and modifier
 *
 * @param modifier modifier to stream
 * @param label labeled message
 * @return std::string resulted stream
 */
std::string StreamExpectedLabeledModifier(const Modifier& modifier,
                                          std::string_view label);

/**
 * @brief Check if the passed labeled-modifier has the same expected
 * configuration
 *
 * @param expect_event expected event
 * @param expect_label expected label
 * @param expect_modifier expected modifier
 * @param actual actual modifier
 * @return testing::AssertionResult
 */
[[nodiscard]] testing::AssertionResult ExpectEqLabeledModifier(
  const EventLevel expect_event, const std::string& expect_label,
  const Modifier& expect_modifier, const LabeledModifier& actual);

/**
 * @brief Check if the passed labeled-modifiers have the expected configuration
 *
 * @param expect expect modifier
 * @param actual actual modifier
 * @return testing::AssertionResult
 */
[[nodiscard]] testing::AssertionResult ExpectEqLabeledModifier(
  const LabeledModifier& expect, const LabeledModifier& actual);

/**
 * @brief Reads all lines from a file
 *
 * @param file_name file name to read from
 * @return std::list<std::string> data written in a file
 */
std::list<std::string> ReadAllLinesFromFile(std::string_view file_name);

/**
 * @brief  compare two list string
 *
 * @param expect list of strings
 * @param actual list of strings
 * @return testing::AssertionResult assertion result
 */
testing::AssertionResult AssertStringList(const std::list<std::string>& expect,
                                          const std::list<std::string>& actual);

/**
 * @brief Create expected message by a logger using name and msg
 *
 * @param logger_name logger name if defined
 * @param msg msg logged msg
 * @param end_msg msg appended at end
 * @return std::string expected logger message
 */
std::string ExpectLoggerMsg(std::string_view logger_name, std::string_view msg,
                            std::string_view end_msg = "\n");

/**
 * @brief Create expected logged message for the passed formatter,
 * LabeledModifier and msg settings
 *
 * @param formater pointer to formater object
 * @param lm labeled modifier
 * @param logger_name logger name if defined
 * @param msg msg logged msg
 * @param end_msg msg appended at end
 * @return std::string expected logger message
 */
std::string ExpectFormattedLoggerMsg(Formatter* formatter,
                                     const LabeledModifier lm,
                                     std::string_view logger_name,
                                     std::string_view msg,
                                     std::string_view end_msg = "\n");
/**
 * @brief Expected logged message by a logger with file writer and
 * TimeLabeledFormatter
 *
 * @param lm labeled modifier
 * @param logger_name logger name if defined
 * @param msg logged message
 * @return std::string expected logger message
 */
std::string ExpectMsgForFileLogger(const LabeledModifier lm,
                                   std::string_view logger_name,
                                   std::string_view msg);

/**
 * @brief Expected logged message by a logger with stream writer and
 * TimeLabeledModifierFormatter
 *
 * @param lm labeled modifier
 * @param logger_name logger name if defined
 * @param msg logged message
 * @return std::string expected logger message
 */
std::string ExpectMsgForStreamLogger(const LabeledModifier lm,
                                     std::string_view logger_name,
                                     std::string_view msg);
#endif  // UTEST_UTILS_HPP_
