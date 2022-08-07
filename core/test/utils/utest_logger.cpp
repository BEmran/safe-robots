#include <gtest/gtest.h>

#include "core/utils/exception.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/terminal.hpp"
#include "core/utils/writter_console.hpp"
#include "core/utils/writter_file.hpp"
#include "utils.hpp"

using namespace core::utils;
const char* g_message = "message";
const char* g_filename = "log.txt";

class TestLogger
{
 public:
  explicit TestLogger(Logger* logger) : logger_(logger), buf_(nullptr)
  {
  }

  ~TestLogger()
  {
    delete buf_;
  }

  void ExpectEqLogNoException(const LabeledModifier& lm, const std::string& msg)
  {
    Clear();
    logger_->Log(lm, msg);
    GetLoggedData();
    ExpectEqLog(msg);
  }

  void ExpectEqLogWithException(const LabeledModifier& lm,
                                const std::string& msg)
  {
    try
    {
      Clear();
      logger_->Log(lm, msg);
    }
    catch (Exception& e)
    {
      GetLoggedData();
    }
    ExpectEqLog(msg);
  }

 private:
  void Clear()
  {
    buf_ = new ConsoleBuffer;
  }

  void ExpectEqLog(const std::string& msg)
  {
    EXPECT_EQ(msg, logs_console_.front());
    EXPECT_EQ(msg, logs_file_.back());
  }

  void GetLoggedData()
  {
    logs_console_ = buf_->restore_cout_buffer();
    logs_file_ = read_all_lines_from_file(g_filename);
  }

  Logger* logger_;
  ConsoleBuffer* buf_;
  std::list<std::string> logs_console_;
  std::list<std::string> logs_file_;
};

TEST(LoggerInformation, Construction)
{
  auto info = LOG_INFORMATION;
  EXPECT_EQ("utest_logger.cpp", info.file);
  EXPECT_EQ("TestBody", info.func);
  EXPECT_EQ(__LINE__ - 3, info.line);
}

TEST(LoggerInformation, ToString)
{
  LogLocation info("filename", "funcname", 1);
  EXPECT_EQ("[filename][funcname][1]", info.ToString());
}

TEST(Logger, ConstructingUsingFilename)
{
  auto* logger = new Logger(g_filename);
  TestLogger test_logger(logger);
  for (const auto& event : EVENTS)
  {
    LabeledModifier lm(event);
    test_logger.ExpectEqLogNoException(lm, g_message);
  }
}

TEST(Logger, ConstructingUsingFormatter)
{
  auto formatter = std::make_shared<NullFormatter>();
  Logger logger(g_filename, formatter, formatter);
  TestLogger test_logger(&logger);
  for (const auto& event : EVENTS)
  {
    LabeledModifier lm(event);
    test_logger.ExpectEqLogNoException(lm, g_message);
  }
}

TEST(Logger, ConstructingUsingException)
{
  auto exception = std::make_shared<ExceptionFactory>("");
  auto* logger = new Logger(g_filename, exception);
  TestLogger test_logger(logger);
  for (const auto& event : EVENTS)
  {
    LabeledModifier lm(event);
    if (event == EventLevel::EL_ERROR)
    {
      test_logger.ExpectEqLogWithException(lm, g_message);
    }
    else
    {
      test_logger.ExpectEqLogNoException(lm, g_message);
    }
  }
}

// TEST(Logger, CreateDefaultLogger) {
//   std::string name = "name";
//   std::string filename = "log.txt";
//   std::string msg = "message";
//   auto logger = CreateDefaultLogger(name, filename);
//   TestLogger test_logger(logger.get());
//   for (size_t i = 0; i < EVENTS.size(); ++i) {
//     LabeledModifier lm(EVENTS[i]);
//     if (EVENTS[i] == EL_ERROR) {
//       test_logger.ExpectEqLogWithException(lm, g_message);
//     } else {
//       test_logger.ExpectEqLogNoException(lm, g_message);
//     }
//   }
// }
