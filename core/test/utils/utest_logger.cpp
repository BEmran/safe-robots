// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/exception.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/modifier.hpp"
#include "core/utils/writer.hpp"
#include "core/utils/writer_file.hpp"
#include "utest/utils.hpp"

using core::utils::CreateNullFormatter;
using core::utils::CreateTimeLabelFormatter;
using core::utils::CreateTimeLabelModifierFormatter;
using core::utils::DebugLabeledModifier;
using core::utils::EventLevel;
using core::utils::Exception;
using core::utils::ExceptionFactory;
using core::utils::Formatter;
using core::utils::Logger;
using core::utils::LoggerConfig;
using core::utils::LogLocation;
using core::utils::Modifier;
using core::utils::Writer;

using namespace std::literals;

constexpr std::string_view kMsg = "message";
constexpr const EventLevel kDebug = EventLevel::DEBUG;
constexpr const EventLevel kError = EventLevel::ERROR;
constexpr const EventLevel kFatal = EventLevel::FATAL;
const LabeledModifier kSimpleLM(kDebug);

TEST(LoggerInformation, Construction) {
  auto info = LOG_INFORMATION;
  EXPECT_EQ("utest_logger.cpp", info.file);
  EXPECT_EQ("TestBody", info.func);
  EXPECT_EQ(__LINE__ - 3, info.line);
}

TEST(LoggerInformation, ToString) {
  LogLocation info("filename", "funcname", 1);
  EXPECT_EQ("[filename][funcname][1]", info.ToString());
}

class MockWriter : public Writer {
 public:
  MockWriter() : Writer(ss) {
  }
  std::string Msg() const {
    std::string str{ss.str()};
    ss.str(std::string(""));
    return str;
  }

 private:
  mutable std::stringstream ss;
  mutable std::string msg_;
};

auto writer1 = std::make_shared<MockWriter>();
auto writer2 = std::make_shared<MockWriter>();
core::utils::WriterFormatterPair wf_pair1{writer1, Formatter()};
core::utils::WriterFormatterPair wf_pair2{writer2, Formatter()};

TEST(Logger, LogWithEvent) {
  LoggerConfig config;
  config.wf_pairs = {wf_pair1};
  Logger logger(config);
  logger.Log(kDebug, kMsg);
  auto expect = wf_pair1.formatter.Format(DebugLabeledModifier(), kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, LogWithLabeledModifier) {
  LoggerConfig config;
  config.wf_pairs = {wf_pair1};
  Logger logger(config);
  logger.Log(DebugLabeledModifier(), kMsg);
  auto expect = wf_pair1.formatter.Format(DebugLabeledModifier(), kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, SingleWriterFormatter) {
  LoggerConfig config;
  config.wf_pairs = {wf_pair1};
  Logger logger(config);
  logger.Log(kSimpleLM, kMsg);
  auto expect = wf_pair1.formatter.Format(kSimpleLM, kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, MultipleWriterFormatter) {
  LoggerConfig config;
  config.wf_pairs = {wf_pair1, wf_pair2};
  Logger logger(config);
  logger.Log(kSimpleLM, kMsg);
  auto expect1 = wf_pair1.formatter.Format(kSimpleLM, kMsg);
  EXPECT_EQ(expect1, writer1->Msg());
  auto expect2 = wf_pair2.formatter.Format(kSimpleLM, kMsg);
  EXPECT_EQ(expect2, writer2->Msg());
}

TEST(Logger, LogWithFormatter) {
  auto format = CreateTimeLabelFormatter();
  LoggerConfig config;
  config.wf_pairs = {{writer1, format}};
  Logger logger(config);
  logger.Log(kSimpleLM, kMsg);
  auto expect = format.Format(kSimpleLM, kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, LogWithName) {
  auto format = CreateTimeLabelFormatter();
  LoggerConfig config;
  config.name = "log-name";
  config.wf_pairs = {{writer1, format}};
  Logger logger(config);
  logger.Log(kSimpleLM, kMsg);
  auto expect = format.Format(kSimpleLM, ExpectLoggerMsg(config.name, kMsg));
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, ExceptionWhenErrorEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  LoggerConfig config;
  config.wf_pairs = {wf_pair1};
  config.expectation_factory = exception;
  Logger logger(config);

  EXPECT_THROW(logger.Log(kError, kMsg), core::utils::Exception);
  auto expect = wf_pair1.formatter.Format(LabeledModifier(kError), kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, ExceptionWhenFatalEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  LoggerConfig config;
  config.wf_pairs = {wf_pair1};
  config.expectation_factory = exception;
  Logger logger(config);

  EXPECT_THROW(logger.Log(kFatal, kMsg), core::utils::Exception);
  auto expect = wf_pair1.formatter.Format(LabeledModifier(kFatal), kMsg);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, CreateStreamLogger) {
  constexpr std::string_view kName = "name";
  std::stringstream ss;
  auto logger = core::utils::CreateStreamLogger(kName, ss);
  auto lm = DebugLabeledModifier();
  logger.Log(lm, kMsg);

  auto expect = ExpectMsgForStreamLogger(lm, kName, kMsg);
  EXPECT_EQ(expect, ss.str());
}

TEST(Logger, CreateFileLogger) {
  constexpr std::string_view kName = "name";
  constexpr std::string_view kFilename = "name_logger.txt";
  auto logger = core::utils::CreateFileLogger(kName, kFilename);
  auto lm = DebugLabeledModifier();
  logger.Log(lm, kMsg);

  const auto file_logged_data = ReadAllLinesFromFile(kFilename);

  auto expect = ExpectMsgForFileLogger(lm, kName, kMsg);
  EXPECT_TRUE(AssertStringList({expect}, file_logged_data));
}

TEST(Logger, CreateStreamAndFileLogger) {
  constexpr std::string_view kName = "name";
  std::stringstream ss;
  constexpr std::string_view kFilename = "name_logger.txt";
  auto logger = core::utils::CreateStreamAndFileLogger(kName, ss, kFilename);
  auto lm = DebugLabeledModifier();
  logger.Log(lm, kMsg);

  auto ss_expect = ExpectMsgForStreamLogger(lm, kName, kMsg);
  EXPECT_EQ(ss_expect, ss.str());

  const auto file_logged_data = ReadAllLinesFromFile(kFilename);
  auto file_expect = ExpectMsgForFileLogger(lm, kName, kMsg);
  EXPECT_TRUE(AssertStringList({file_expect}, file_logged_data));
}
