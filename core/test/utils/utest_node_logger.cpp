// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/node_logger.hpp"
#include "utest/utils.hpp"

using core::utils::DebugLabeledModifier;
using core::utils::DebugModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::ErrorModifier;
using core::utils::InfoLabeledModifier;
using core::utils::InfoModifier;
using core::utils::LabeledModifier;
using core::utils::Logger;
using core::utils::NodeLabeledModifiers;
using core::utils::NodeLogger;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

const LabeledModifier kDebugLm(EventLevel::EL_DEBUG, DebugModifier());
const LabeledModifier kErrorLm(EventLevel::EL_ERROR, ErrorModifier());
const LabeledModifier kInfoLm(EventLevel::EL_INFO, InfoModifier());
const LabeledModifier kWarnLm(EventLevel::EL_WARN, WarnModifier());
const NodeLabeledModifiers kNodeLm(kDebugLm, kErrorLm, kInfoLm, kWarnLm);

constexpr const char* kMessage = "message";
constexpr const char* kHeader = "Header";
constexpr const char* kHeaderMessage = "[Header]: message";

class MockLogger : public Logger {
 public:
  MockLogger() : Logger({}) {
  }
  void Log(const LabeledModifier& lm, const std::string& msg) override {
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
  LabeledModifier lm_{DebugLabeledModifier()};
  std::string msg_;
};

const auto kMockLogger = std::make_shared<MockLogger>();

// check creating NodeLabel Modifier
TEST(NodeLabeledModifiers, Construct) {
  ExpectEqLabeledModifier(kDebugLm, kNodeLm.debug);
  ExpectEqLabeledModifier(kErrorLm, kNodeLm.error);
  ExpectEqLabeledModifier(kInfoLm, kNodeLm.info);
  ExpectEqLabeledModifier(kWarnLm, kNodeLm.warn);
}

// check creating Default NodeLabel Modifier using
// CreateSystemNodeLabelModifier function
TEST(NodeLabeledModifiers, DefaultConstruct) {
  const NodeLabeledModifiers default_node_lm;
  ExpectEqLabeledModifier(default_node_lm.debug, DebugLabeledModifier());
  ExpectEqLabeledModifier(default_node_lm.error, ErrorLabeledModifier());
  ExpectEqLabeledModifier(default_node_lm.info, InfoLabeledModifier());
  ExpectEqLabeledModifier(default_node_lm.warn, WarnLabeledModifier());
}

// Log using NodeLogger with default header
TEST(MockLogger, LogWithDefaultHeaderAndLM) {
  auto n_logger = std::make_shared<NodeLogger>(kMockLogger);
  n_logger->SetLogHeader(kHeader);

  n_logger->LogDebug(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(DebugLabeledModifier(), kMockLogger->LM());

  n_logger->LogWarn(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(WarnLabeledModifier(), kMockLogger->LM());

  n_logger->LogInfo(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(InfoLabeledModifier(), kMockLogger->LM());

  n_logger->LogError(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(ErrorLabeledModifier(), kMockLogger->LM());
}

// Log using NodeLogger with special header and default LM
TEST(MockLogger, LogWithHeaderAndDefaultLM) {
  auto n_logger = std::make_shared<NodeLogger>(kMockLogger);
  n_logger->SetLogHeader(kHeader);
  n_logger->LogInfo(kMessage);

  n_logger->LogDebug(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(DebugLabeledModifier(), kMockLogger->LM());

  n_logger->LogWarn(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(WarnLabeledModifier(), kMockLogger->LM());

  n_logger->LogInfo(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(InfoLabeledModifier(), kMockLogger->LM());

  n_logger->LogError(kMessage);
  EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
  ExpectEqLabeledModifier(ErrorLabeledModifier(), kMockLogger->LM());
}

// construct with special LM
TEST(MockLogger, ConstructWithSpecialLM) {
  auto n_logger = std::make_shared<NodeLogger>(kMockLogger, kNodeLm);

  n_logger->LogDebug(kMessage);
  ExpectEqLabeledModifier(kNodeLm.debug, kMockLogger->LM());

  n_logger->LogError(kMessage);
  ExpectEqLabeledModifier(kNodeLm.error, kMockLogger->LM());

  n_logger->LogInfo(kMessage);
  ExpectEqLabeledModifier(kNodeLm.info, kMockLogger->LM());

  n_logger->LogWarn(kMessage);
  ExpectEqLabeledModifier(kNodeLm.warn, kMockLogger->LM());
}

TEST(CreateFileAndConsoleLogger, CheckInitializedWriters) {
  constexpr auto kName = "name";
  constexpr auto kFilename = "name_logger.txt";
  auto n_logger = core::utils::CreateNodeLogger(kName);

  ConsoleBuffer c_buf;
  n_logger->LogDebug(kMessage);
  EXPECT_TRUE(
    AssertFileAndConsole(kFilename, c_buf, DebugLabeledModifier(), kMessage));
}

TEST(CreateSystemNodeLogger, CheckInitializedWriters) {
  constexpr auto kFilename = "sys_logger.txt";
  const auto n_logger = core::utils::CreateSystemNodeLogger("");
  ConsoleBuffer c_buf;
  n_logger->LogDebug(kMessage);
  EXPECT_TRUE(
    AssertFileAndConsole(kFilename, c_buf, DebugLabeledModifier(), kMessage));
}
