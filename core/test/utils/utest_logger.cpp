// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/exception.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/modifier.hpp"
#include "core/utils/writer.hpp"
#include "core/utils/writer_file.hpp"
#include "utest/utils.hpp"

using core::utils::EventLevel;
using core::utils::Exception;
using core::utils::ExceptionFactory;
using core::utils::FormaterInterface;
using core::utils::Logger;
using core::utils::LogLocation;
using core::utils::Modifier;
using core::utils::NullFormater;

constexpr const char* kMessage = "message";
constexpr const EventLevel debug = EventLevel::DEBUG;
constexpr const EventLevel error = EventLevel::ERROR;
constexpr const EventLevel fatal = EventLevel::FATAL;
const LabeledModifier simple_lm(debug, Modifier());
const LabeledModifier debug_lm(debug);

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

class MockWriter : public core::utils::Writer {
 public:
  MockWriter() : core::utils::Writer(ss) {
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
auto formater1 = std::make_shared<NullFormater>();
auto formater2 = std::make_shared<NullFormater>();
core::utils::WriterFormatterPair wf_pair1{writer1, formater1};
core::utils::WriterFormatterPair wf_pair2{writer2, formater2};

TEST(Logger, LogWithEvent) {
  Logger logger({wf_pair1});
  logger.Log(debug, kMessage);
  const auto expect = EventLevelToString(debug) + " " + kMessage + "\n";
  EXPECT_EQ(expect, writer1->Msg());
}
TEST(Logger, LogWithLabeledModifier) {
  Logger logger({wf_pair1});
  logger.Log(debug_lm, kMessage);
  const auto expect = debug_lm.ToString() + " " + kMessage + "\n";
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, SingleWriterFormatter) {
  Logger logger({wf_pair1});
  logger.Log(simple_lm, kMessage);
  const auto expect = simple_lm.ToString() + " " + kMessage + "\n";
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, MultipleWriterFormatter) {
  Logger logger({wf_pair1, wf_pair2});
  logger.Log(simple_lm, kMessage);
  const auto expect = simple_lm.ToString() + " " + kMessage + "\n";
  EXPECT_EQ(expect, writer1->Msg());
  EXPECT_EQ(expect, writer2->Msg());
}

TEST(Logger, ExceptionWhenErrorEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  Logger logger({wf_pair1}, exception);

  const auto expect = EventLevelToString(error) + " " + kMessage + "\n";
  EXPECT_THROW(logger.Log(error, kMessage), core::utils::Exception);
  EXPECT_EQ(expect, writer1->Msg());
}

TEST(Logger, ExceptionWheFatalEvent) {
  auto exception = std::make_shared<ExceptionFactory>("");
  Logger logger({wf_pair1}, exception);

  const auto expect = EventLevelToString(fatal) + " " + kMessage + "\n";
  EXPECT_THROW(logger.Log(fatal, kMessage), core::utils::Exception);
  EXPECT_EQ(expect, writer1->Msg());
}

// TEST(CreateFileAndConsoleLogger, CheckInitializedWriters) {
//   constexpr auto kName = "name";
//   constexpr auto kFilename = "name_logger.txt";
//   auto logger = core::utils::CreateFileAndConsoleLogger(kName);

//   ConsoleBuffer c_buf;
//   logger->Log(event, kMessage);
//   EXPECT_TRUE(
//     AssertFileAndConsole(kFilename, c_buf, labeled_modifier, kMessage));
// }
