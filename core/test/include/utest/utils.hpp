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
#include "core/utils/modifier.hpp"
#include "gtest/gtest.h"

using core::utils::EventLevel;
using core::utils::LabeledModifier;
using core::utils::Modifier;
namespace BG = core::utils::BG;
namespace FG = core::utils::FG;
namespace FMT = core::utils::FMT;

// carate list of available variables
constexpr const char* kLabels[] = {"DEBUG", "ERROR", "FATAL", "INFO", "WARN"};
// should be same order as kLabels
const std::vector<EventLevel> kEvents = {EventLevel::DEBUG, EventLevel::ERROR,
                                         EventLevel::FATAL, EventLevel::INFO,
                                         EventLevel::WARN};

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

// convert Modifier to a string
std::string ModifierToString(const std::vector<int>& options);

// check if the passed modifier has the passed configuration
void ExpectEqModifier(const std::vector<int>& expect_options,
                      const Modifier& actual);

// check if the passed modifiers have the same configuration
void ExpectEqModifier(const Modifier& expect, const Modifier& actual);

// generate a string using stream method of the passed label and modifier
std::string StreamExpectedLabeledModifier(std::string_view label,
                                          const Modifier& modifier);

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
std::list<std::string> ReadAllLinesFromFile(std::string_view file_name);

/**
 * @brief  compare two list string
 *
 * @param expect list 1
 * @param actual list 2
 * @return testing::AssertionResult assertion result
 */
testing::AssertionResult AssertStringList(const std::list<std::string>& expect,
                                          const std::list<std::string>& actual);

#endif  // UTEST_UTILS_HPP_
