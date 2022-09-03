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
constexpr const char* kFilename = "log.txt";
const auto kDebugLM = core::utils::DebugLabeledModifier();
const auto kErrorLM = core::utils::ErrorLabeledModifier();

class TestLogger {
 public:
  explicit TestLogger(Logger* logger) : logger_(logger), buf_(nullptr) {
  }

  ~TestLogger() {
    delete buf_;
  }

  void ExpectEqLogNoException(const LabeledModifier& lm,
                              const std::string& msg) {
    Clear();
    logger_->Log(lm, msg);
    GetLoggedData();
    ExpectEqLog(msg);
  }

  void ExpectEqLogWithException(const LabeledModifier& lm,
                                const std::string& msg) {
    try {
      Clear();
      logger_->Log(lm, msg);
    } catch (Exception& e) {
      GetLoggedData();
    }
    ExpectEqLog(msg);
  }

 private:
  void Clear() {
    buf_ = new ConsoleBuffer;
  }

  void ExpectEqLog(const std::string& msg) {
    EXPECT_EQ(msg, logs_console_.front());
    EXPECT_EQ(msg, logs_file_.back());
  }

  void GetLoggedData() {
    logs_console_ = buf_->RestoreCoutBuffer();
    logs_file_ = ReadAllLinesFromFile(kFilename);
  }

  Logger* logger_;
  ConsoleBuffer* buf_;
  std::list<std::string> logs_console_;
  std::list<std::string> logs_file_;
};

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

TEST(Logger, ConstructingUsingFilename) {
  auto* logger = new Logger(kFilename);
  TestLogger test_logger(logger);
  test_logger.ExpectEqLogNoException(kDebugLM, kMessage);
}

TEST(Logger, ConstructingUsingFormatter) {
  auto formatter = core::utils::CreateNullFormatter();
  auto* logger = new Logger(kFilename, formatter, formatter);
  TestLogger test_logger(logger);
  test_logger.ExpectEqLogNoException(kDebugLM, kMessage);
}

TEST(Logger, ConstructingUsingException) {
  auto exception = std::make_shared<ExceptionFactory>("");
  auto* logger = new Logger(kFilename, exception);
  TestLogger test_logger(logger);
  test_logger.ExpectEqLogNoException(kDebugLM, kMessage);
  test_logger.ExpectEqLogWithException(kErrorLM, kMessage);
}

// TEST(Logger, CreateDefaultLogger) {
//   std::string name = "name";
//   std::string filename = "log.txt";
//   std::string msg = "message";
//   auto logger = CreateDefaultLogger(name, filename);
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
