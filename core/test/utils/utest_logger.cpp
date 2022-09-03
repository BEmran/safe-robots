// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/exception.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/terminal.hpp"
#include "core/utils/writer_console.hpp"
#include "core/utils/writer_file.hpp"
#include "utest/utils.hpp"

using core::utils::Exception;
using core::utils::ExceptionFactory;
using core::utils::Logger;
using core::utils::LogLocation;

constexpr const char* kMessage = "message";
const auto kDebugLM = core::utils::DebugLabeledModifier();
const auto kErrorLM = core::utils::ErrorLabeledModifier();

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
  void Dump(const std::string& str) const override {
    msg_ = str;
  }
  std::string Msg() const {
    return msg_;
  }

 private:
  mutable std::string msg_;
};

auto writer1 = std::make_shared<MockWriter>();
auto writer2 = std::make_shared<MockWriter>();
core::utils::WriterFormatterPair wf_pair1{writer1, core::utils::Formatter()};
core::utils::WriterFormatterPair wf_pair2{writer2, core::utils::Formatter()};

TEST(Logger, SingleWriterFormatter) {
  Logger logger({wf_pair1});
  logger.Log(kDebugLM, kMessage);
  EXPECT_EQ(kMessage, writer1->Msg());
}

TEST(Logger, MultipleWriterFormatter) {
  Logger logger({wf_pair1, wf_pair2});
  logger.Log(kDebugLM, kMessage);
  EXPECT_EQ(kMessage, writer1->Msg());
  EXPECT_EQ(kMessage, writer2->Msg());
}

TEST(Logger, WithException) {
  auto exception = std::make_shared<ExceptionFactory>("");
  Logger logger({wf_pair1}, exception);

  logger.Log(kDebugLM, kMessage);
  EXPECT_EQ(kMessage, writer1->Msg());
  EXPECT_THROW(logger.Log(kErrorLM, kMessage), core::utils::Exception);
  EXPECT_EQ(kMessage, writer1->Msg());
}

TEST(CreateFileAndConsoleLogger, CheckInitializedWriters) {
  constexpr auto kName = "name";
  constexpr auto kFilename = "name_logger.txt";
  auto logger = core::utils::CreateFileAndConsoleLogger(kName);

  ConsoleBuffer c_buf;
  logger->Log(kDebugLM, kMessage);
  EXPECT_TRUE(AssertFileAndConsole(kFilename, c_buf, kDebugLM, kMessage));
}
