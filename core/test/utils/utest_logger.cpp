#include <gtest/gtest.h>

#include "core/utils/exception.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/terminal.hpp"
#include "core/utils/writter_console.hpp"
#include "core/utils/writter_file.hpp"
#include "utest/utils.hpp"

using namespace core::utils;
const char* g_message = "message";
const char* g_filename = "log.txt";

struct TestLogger
{
  Logger* m_logger;
  ConsoleBuffer* m_buf;
  std::list<std::string> m_logs_console;
  std::list<std::string> m_logs_file;
  explicit TestLogger(Logger* logger) : m_logger(logger), m_buf(nullptr)
  {
  }

  ~TestLogger()
  {
    delete m_buf;
  }

  void expect_eq_log_no_exception(const LabeledModifier& lm,
                                  const std::string& msg)
  {
    clear();
    m_logger->log(lm, msg);
    get_logged_data();
    expect_eq_log(msg);
  }

  void expect_eq_log_with_exception(const LabeledModifier& lm,
                                    const std::string& msg)
  {
    try
    {
      clear();
      m_logger->log(lm, msg);
    }
    catch (Exception& e)
    {
      get_logged_data();
    }
    expect_eq_log(msg);
  }

  void clear()
  {
    m_buf = new ConsoleBuffer;
  }

  void expect_eq_log(const std::string& msg)
  {
    EXPECT_EQ(msg, m_logs_console.front());
    EXPECT_EQ(msg, m_logs_file.back());
  }

  void get_logged_data()
  {
    m_logs_console = m_buf->restore_cout_buffer();
    m_logs_file = read_all_lines_from_file(g_filename);
  }
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
  EXPECT_EQ("[filename][funcname][1]", info.to_string());
}

TEST(Logger, ConstructingUsingFilename)
{
  auto logger = new Logger(g_filename);
  TestLogger test_logger(logger);
  for (const auto& event : EVENTS)
  {
    LabeledModifier lm(event);
    test_logger.expect_eq_log_no_exception(lm, g_message);
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
    test_logger.expect_eq_log_no_exception(lm, g_message);
  }
}

TEST(Logger, ConstructingUsingException)
{
  auto exception = std::make_shared<ExceptionFactory>("");
  auto logger = new Logger(g_filename, exception);
  TestLogger test_logger(logger);
  for (const auto& event : EVENTS)
  {
    LabeledModifier lm(event);
    if (event == event_level_t::EL_ERROR)
    {
      test_logger.expect_eq_log_with_exception(lm, g_message);
    }
    else
    {
      test_logger.expect_eq_log_no_exception(lm, g_message);
    }
  }
}

// TEST(Logger, CreateDefaultLogger) {
//   std::string name = "name";
//   std::string filename = "log.txt";
//   std::string msg = "message";
//   auto logger = create_default_logger(name, filename);
//   TestLogger test_logger(logger.get());
//   for (size_t i = 0; i < EVENTS.size(); ++i) {
//     LabeledModifier lm(EVENTS[i]);
//     if (EVENTS[i] == event_level_t::EL_ERROR) {
//       test_logger.expect_eq_log_with_exception(lm, g_message);
//     } else {
//       test_logger.expect_eq_log_no_exception(lm, g_message);
//     }
//   }
// }
