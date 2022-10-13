// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <dependencies/simple_logger.hpp>
#include <sstream>
#include <string_view>

constexpr std::string_view HEADER = "HEADER";
using depend::Debug;
using depend::Error;
using depend::Log;
using depend::Logger;
using namespace std::literals::string_literals;

TEST(Logger, DefaultTest) {
  const std::string msg = "msg 1";
  Logger logger;
  logger << msg << std::endl;
  EXPECT_TRUE(true);
}

TEST(Logger, DebugTest) {
  const std::string msg = "debug msg";
  Debug() << msg;
  EXPECT_TRUE(true);
}

TEST(Logger, ErrorTest) {
  const std::string msg = "error msg";
  Error() << msg;
  EXPECT_TRUE(true);
}

TEST(Logger, LogUsingLogFunctionAppendEndLine) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  Log("", ss, cb) << "";
  const std::string expect = ": "s + "\n";
  EXPECT_EQ(expect, ss.str());
}

TEST(Logger, LogUsingObjectDoesNotAppendEndLine) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  Logger logger("", ss, cb);
  logger << "";
  const std::string expect = ": "s;
  EXPECT_EQ(expect, ss.str());
}

TEST(Logger, DoesNotAppendEndLineWhenDestructedAndNotUsed) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  auto logger = new Logger("", ss, cb);
  delete logger;
  EXPECT_TRUE(ss.str().empty());
}

TEST(Logger, DoesNotAppendEndLineWhenDestructedAndPreviousMessageHasEndLine) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  auto logger = new Logger("", ss, cb);
  *logger << std::endl;
  EXPECT_EQ("\n", ss.str());
  ss.str("");  // clear stringstream
  delete logger;
  EXPECT_TRUE(ss.str().empty());
}

TEST(Logger, AppendEndLineWhenDestructedAndPreviousMessageDoesNotHaveEndLine) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  auto logger = new Logger("", ss, cb);
  *logger << "";
  EXPECT_EQ(": ", ss.str());
  ss.str("");  // clear stringstream
  delete logger;
  EXPECT_EQ("\n", ss.str());
}

TEST(Logger, LogUsingLogFunction) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  const std::string msg = "msg 1";
  Log(HEADER, ss, cb) << msg;
  const std::string expect = HEADER.data() + ": "s + msg + "\n";
  EXPECT_EQ(expect, ss.str());
}

TEST(Logger, ConstructLoggerObject) {
  std::stringstream ss;
  auto cb = [] { return std::string{}; };
  const std::string msg = "msg 1";
  Logger logger(HEADER, ss, cb);
  logger << msg;
  const std::string expect = HEADER.data() + ": "s + msg;
  EXPECT_EQ(expect, ss.str());
}
