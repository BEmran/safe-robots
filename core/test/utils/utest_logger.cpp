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
using core::utils::LogLocation;
using core::utils::Modifier;
using core::utils::Writer;

using namespace std::literals;

constexpr std::string_view kMessage = "message";
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

std::string ExpectMessage(const EventLevel event) {
  return "["s + EventLevelToString(event) + "] "s + kMessage.data();
}

std::string ExpectMessage(const LabeledModifier lm) {
  return "["s + lm.ToString() + "] "s + kMessage.data();
}

// std::string ExpectMessageWithTimeFormater(std::string_view logger_name,
//                                           const EventLevel event) {
//   auto formater = CreateTimeLabelModifierFormatter();
//   return formater.Format("");
// }

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
  Logger logger({wf_pair1});
  logger.Log(kDebug, kMessage);
  auto expect = wf_pair1.formatter.Format(DebugLabeledModifier(), kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, LogWithLabeledModifier) {
  Logger logger({wf_pair1});
  logger.Log(DebugLabeledModifier(), kMessage);
  auto expect = wf_pair1.formatter.Format(DebugLabeledModifier(), kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, SingleWriterFormatter) {
  Logger logger({wf_pair1});
  logger.Log(kSimpleLM, kMessage);
  auto expect = wf_pair1.formatter.Format(kSimpleLM, kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, MultipleWriterFormatter) {
  Logger logger({wf_pair1, wf_pair2});
  logger.Log(kSimpleLM, kMessage);
  auto expect1 = wf_pair1.formatter.Format(kSimpleLM, kMessage);
  EXPECT_EQ(expect1, writer1->Msg());
  auto expect2 = wf_pair2.formatter.Format(kSimpleLM, kMessage);
  EXPECT_EQ(expect2, writer2->Msg());
}

TEST(Logger, LogWithFormater) {
  auto format = CreateTimeLabelFormatter();
  Logger logger({{writer1, format}});
  logger.Log(kSimpleLM, kMessage);
  auto expect = format.Format(kSimpleLM, kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, ExceptionWhenErrorEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  Logger logger({wf_pair1}, exception);

  EXPECT_THROW(logger.Log(kError, kMessage), core::utils::Exception);
  auto expect = wf_pair1.formatter.Format(LabeledModifier(kError), kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, ExceptionWheFatalEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  Logger logger({wf_pair1}, exception);

  EXPECT_THROW(logger.Log(kFatal, kMessage), core::utils::Exception);
  auto expect = wf_pair1.formatter.Format(LabeledModifier(kFatal), kMessage);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, CreateStreamLogger) {
  constexpr auto kName = "name";
  std::stringstream ss;
  auto logger = core::utils::CreateStreamLogger(kName, ss);
  logger.Log(DebugLabeledModifier(), kMessage);
  auto formater = CreateTimeLabelModifierFormatter();
  auto expect = formater.Format(DebugLabeledModifier(), kMessage);
  EXPECT_EQ(expect, ss.str());
}

TEST(Logger, CreateFileLogger) {
  constexpr auto kName = "name";
  constexpr auto kFilename = "name_logger.txt";
  auto logger = core::utils::CreateFileLogger(kName, kFilename);
  logger.Log(DebugLabeledModifier(), kMessage);

  const auto f_logged_data = ReadAllLinesFromFile(kFilename);
  auto formater = CreateTimeLabelFormatter();
  auto expect = formater.Format(DebugLabeledModifier(), kMessage);
  EXPECT_TRUE(AssertStringList({expect}, f_logged_data));
}

TEST(Logger, CreateStreamAndFileLogger) {
  constexpr auto kName = "name";
  std::stringstream ss;
  constexpr auto kFilename = "name_logger.txt";
  auto logger = core::utils::CreateStreamAndFileLogger(kName, ss, kFilename);
  logger.Log(DebugLabeledModifier(), kMessage);

  const auto f_logged_data = ReadAllLinesFromFile(kFilename);
  auto ss_formater = CreateTimeLabelModifierFormatter();
  auto file_formater = CreateTimeLabelFormatter();
  auto ss_expect = ss_formater.Format(DebugLabeledModifier(), kMessage);
  auto file_expect = file_formater.Format(DebugLabeledModifier(), kMessage);
  EXPECT_EQ(ss_expect, ss.str());
  EXPECT_TRUE(AssertStringList({file_expect}, f_logged_data));
}
