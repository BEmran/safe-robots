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

// class TestLogger {
//  public:
//   explicit TestLogger(Logger* logger) : logger_(logger), buf_(nullptr) {
//   }

//   ~TestLogger() {
//     delete buf_;
//   }

//   void ExpectEqLogNoException(const LabeledModifier& lm,
//                               const std::string& msg) {
//     Clear();
//     logger_->Log(lm, msg);
//     GetLoggedData();
//     ExpectEqLog(msg);
//   }

//   void ExpectEqLogWithException(const LabeledModifier& lm,
//                                 const std::string& msg) {
//     try {
//       Clear();
//       logger_->Log(lm, msg);
//     } catch (Exception& e) {
//       GetLoggedData();
//     }
//     ExpectEqLog(msg);
//   }

//  private:
//   void Clear() {
//     buf_ = new ConsoleBuffer;
//   }

//   void ExpectEqLog(const std::string& msg) {
//     EXPECT_EQ(msg, logs_console_.front());
//     EXPECT_EQ(msg, logs_file_.back());
//   }

//   void GetLoggedData() {
//     logs_console_ = buf_->RestoreCoutBuffer();
//     logs_file_ = ReadAllLinesFromFile(kFilename);
//   }

//   Logger* logger_;
//   ConsoleBuffer* buf_;
//   std::list<std::string> logs_console_;
//   std::list<std::string> logs_file_;
// };

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
  MockWriter() = default;
  ~MockWriter() override = default;
  void Dump(const std::string& str) const override {
    msg = str;
  }
  std::string Msg() const {
    return msg;
  }

 private:
  mutable std::string msg;
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

// TEST(Logger, CreateFileAndConsoleLogger) {
//   std::string name = "name";
//   std::string filename = "log.txt";
//   std::string msg = "message";
//   auto logger = CreateFileAndConsoleLogger(name, filename);
//   TestLogger test_logger(logger.get());
//   for (size_t i = 0; i < kEvents.size(); ++i) {
//     LabeledModifier lm(kEvents[i]);
//     if (kEvents[i] == EL_ERROR) {
//       test_logger.ExpectEqLogWithException(lm, kMessage);
//     } else {
//       test_logger.ExpectEqLogNoException(lm, kMessage);
//     }
//   }
// }
