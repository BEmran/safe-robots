// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <string_view>

#include "core/utils/logger_node.hpp"
#include "utils.hpp"

using namespace std::literals;

using core::utils::DebugLabeledModifier;
using core::utils::DebugModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::ErrorModifier;
using core::utils::FatalLabeledModifier;
using core::utils::FatalModifier;
using core::utils::InfoLabeledModifier;
using core::utils::InfoModifier;
using core::utils::LabeledModifier;
using core::utils::Logger;
using core::utils::LoggerConfig;
using core::utils::LoggerLabeledModifiers;
using core::utils::NodeLogger;
using core::utils::StreamLogger;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

const LabeledModifier kDebugLm(EventLevel::DEBUG, DebugModifier());
const LabeledModifier kErrorLm(EventLevel::ERROR, ErrorModifier());
const LabeledModifier kFatalLm(EventLevel::FATAL, FatalModifier());
const LabeledModifier kInfoLm(EventLevel::INFO, InfoModifier());
const LabeledModifier kWarnLm(EventLevel::WARN, WarnModifier());
const LoggerLabeledModifiers kNodeLm(kDebugLm, kErrorLm, kFatalLm, kInfoLm,
                                     kWarnLm);

constexpr std::string_view kMessage = "message";
constexpr std::string_view kHeader = "Header";

class MockLogger : public Logger {
 public:
  MockLogger() : Logger({}) {
  }
  void Log(const LabeledModifier& lm, std::string_view msg) const override {
    lm_ = lm;
    msg_ = msg;
  }
  inline std::string Msg() const {
    return msg_;
  }
  inline LabeledModifier LM() const {
    return lm_;
  }

 private:
  mutable LabeledModifier lm_{DebugLabeledModifier()};
  mutable std::string msg_;
};

const auto kMockLogger = std::make_shared<MockLogger>();

// check creating NodeLabel Modifier
TEST(LoggerLabeledModifiers, Construct) {
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kNodeLm.debug));
  EXPECT_TRUE(ExpectEqLabeledModifier(kErrorLm, kNodeLm.error));
  EXPECT_TRUE(ExpectEqLabeledModifier(kFatalLm, kNodeLm.fatal));
  EXPECT_TRUE(ExpectEqLabeledModifier(kInfoLm, kNodeLm.info));
  EXPECT_TRUE(ExpectEqLabeledModifier(kWarnLm, kNodeLm.warn));
}

// check creating Default NodeLabel Modifier using
TEST(LoggerLabeledModifiers, DefaultConstruct) {
  const LoggerLabeledModifiers default_node_lm;
  EXPECT_TRUE(
    ExpectEqLabeledModifier(default_node_lm.debug, DebugLabeledModifier()));
  EXPECT_TRUE(
    ExpectEqLabeledModifier(default_node_lm.error, ErrorLabeledModifier()));
  EXPECT_TRUE(
    ExpectEqLabeledModifier(default_node_lm.fatal, FatalLabeledModifier()));
  EXPECT_TRUE(
    ExpectEqLabeledModifier(default_node_lm.info, InfoLabeledModifier()));
  EXPECT_TRUE(
    ExpectEqLabeledModifier(default_node_lm.warn, WarnLabeledModifier()));
}

// Log using NodeLogger with default header
TEST(MockLogger, LogWithDefaultHeaderAndLMs) {
  NodeLogger n_logger(kMockLogger);

  n_logger.Debug(kMessage);
  EXPECT_EQ(kMessage, kMockLogger->Msg());
  EXPECT_TRUE(
    ExpectEqLabeledModifier(DebugLabeledModifier(), kMockLogger->LM()));

  n_logger.Error(kMessage);
  EXPECT_EQ(kMessage, kMockLogger->Msg());
  EXPECT_TRUE(
    ExpectEqLabeledModifier(ErrorLabeledModifier(), kMockLogger->LM()));

  n_logger.Fatal(kMessage);
  EXPECT_EQ(kMessage, kMockLogger->Msg());
  EXPECT_TRUE(
    ExpectEqLabeledModifier(FatalLabeledModifier(), kMockLogger->LM()));

  n_logger.Warn(kMessage);
  EXPECT_EQ(kMessage, kMockLogger->Msg());
  EXPECT_TRUE(
    ExpectEqLabeledModifier(WarnLabeledModifier(), kMockLogger->LM()));

  n_logger.Info(kMessage);
  EXPECT_EQ(kMessage, kMockLogger->Msg());
  EXPECT_TRUE(
    ExpectEqLabeledModifier(InfoLabeledModifier(), kMockLogger->LM()));
}

// Log using NodeLogger with special header and default LM
TEST(MockLogger, LogWithHeaderAndDefaultLM) {
  std::stringstream ss;
  auto writer = std::make_shared<core::utils::Writer>(ss);
  auto formatter = core::utils::Formatter();
  LoggerConfig config;
  config.wf_pairs = {{writer, formatter}};
  auto logger = std::make_shared<Logger>(config);
  NodeLogger n_logger(logger);
  n_logger.SetLabel(kHeader);

  n_logger.Debug(kMessage);
  std::stringstream expect;
  expect << formatter.Format(DebugLabeledModifier(), "") << "[" << kHeader
         << "] " << kMessage << "\n";
  EXPECT_EQ(expect.str(), ss.str());
}

// Log using NodeLogger with special header and default LM
TEST(MockLogger, LogWithDefaultHeaderAndDefaultLM) {
  std::stringstream ss;
  auto writer = std::make_shared<core::utils::Writer>(ss);
  auto formatter = core::utils::Formatter();
  LoggerConfig config;
  config.wf_pairs = {{writer, formatter}};
  auto logger = std::make_shared<Logger>(config);
  NodeLogger n_logger(logger);

  n_logger.Info() << kMessage;
  std::stringstream expect;
  expect << formatter.Format(InfoLabeledModifier(), "") << kMessage << "\n";
  EXPECT_EQ(expect.str(), ss.str());
}

// construct with special LM
TEST(MockLogger, ConstructWithSpecialLM) {
  LoggerLabeledModifiers lms(kDebugLm, kDebugLm, kDebugLm, kDebugLm, kDebugLm);
  NodeLogger n_logger(kMockLogger, lms);

  n_logger.Debug(kMessage);
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kMockLogger->LM()));

  n_logger.Error(kMessage);
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kMockLogger->LM()));

  n_logger.Fatal(kMessage);
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kMockLogger->LM()));

  n_logger.Info(kMessage);
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kMockLogger->LM()));

  n_logger.Warn(kMessage);
  EXPECT_TRUE(ExpectEqLabeledModifier(kDebugLm, kMockLogger->LM()));
}

TEST(CreateNodeLoggerUsingSystemLoggerLogger, CheckInitializedWriters) {
  constexpr auto kFilename = "sys_logger.txt";
  constexpr std::string_view kNodeName = "node1";
  const auto n_logger =
    core::utils::CreateNodeLoggerUsingSystemLogger(kNodeName);
  n_logger.Debug(kMessage);

  const auto actual = ReadAllLinesFromFile(kFilename);
  auto expect = ExpectMsgForFileLogger(DebugLabeledModifier(),
                                       "sys] ["s + kNodeName.data(), kMessage);
  EXPECT_TRUE(AssertStringList({expect}, actual));
}
