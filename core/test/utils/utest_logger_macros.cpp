// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/logger_macros.hpp"
#include "utest/utils.hpp"

using core::utils::DebugLabeledModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::FatalLabeledModifier;
using core::utils::InfoLabeledModifier;
using core::utils::Logger;
using core::utils::WarnLabeledModifier;

constexpr std::string_view kMsg = "message";
constexpr std::string_view kName = "sys";
constexpr std::string_view kFilename = "sys_logger.txt";

TEST(SysLogger, SystemDebugMacros) {
  SYS_LOG_DEBUG(kMsg);
  EXPECT_THROW(SYS_LOG_ERROR(kMsg), core::utils::Exception);
  EXPECT_THROW(SYS_LOG_FATAL(kMsg), core::utils::Exception);
  SYS_LOG_INFO(kMsg);
  SYS_LOG_WARN(kMsg);

  const auto file_logged_data = ReadAllLinesFromFile(kFilename);

  auto debug_expect_msg =
    ExpectMsgForFileLogger(DebugLabeledModifier(), kName, kMsg);
  auto error_expect_msg =
    ExpectMsgForFileLogger(ErrorLabeledModifier(), kName, kMsg);
  auto fatal_expect_msg =
    ExpectMsgForFileLogger(FatalLabeledModifier(), kName, kMsg);
  auto info_expect_msg =
    ExpectMsgForFileLogger(InfoLabeledModifier(), kName, kMsg);
  auto warn_expect_msg =
    ExpectMsgForFileLogger(WarnLabeledModifier(), kName, kMsg);

  EXPECT_TRUE(
    AssertStringList({debug_expect_msg, error_expect_msg, fatal_expect_msg,
                      info_expect_msg, warn_expect_msg},
                     file_logged_data));
}
