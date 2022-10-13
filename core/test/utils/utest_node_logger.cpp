// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/node_logger.hpp"
#include "utest/utils.hpp"

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
using core::utils::NodeLabeledModifiers;
using core::utils::NodeLogger;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

const LabeledModifier kDebugLm(EventLevel::DEBUG, DebugModifier());
const LabeledModifier kErrorLm(EventLevel::ERROR, ErrorModifier());
const LabeledModifier kFatalLm(EventLevel::FATAL, FatalModifier());
const LabeledModifier kInfoLm(EventLevel::INFO, InfoModifier());
const LabeledModifier kWarnLm(EventLevel::WARN, WarnModifier());
const NodeLabeledModifiers kNodeLm(kDebugLm, kErrorLm, kFatalLm, kInfoLm,
                                   kWarnLm);

constexpr std::string_view kMessage = "message";
constexpr std::string_view kHeader = "Header";
constexpr std::string_view kHeaderMessage = "[Header]: message";

class MockLogger : public Logger {
 public:
  MockLogger() : Logger({}) {
  }
  void Log(const LabeledModifier& lm, std::string_view msg) const override {
    // lm_ = lm;
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

// // Log using NodeLogger with default header
// TEST(MockLogger, LogWithDefaultHeaderAndLM) {
//   auto n_logger = std::make_shared<Logger>(kMockLogger);
//   // n_logger->SetLogHeader(kHeader);

//   n_logger->Debug(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(DebugLabeledModifier(), kMockLogger->LM());

//   n_logger->Warn(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(WarnLabeledModifier(), kMockLogger->LM());

//   n_logger->Info(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(InfoLabeledModifier(), kMockLogger->LM());

//   n_logger->Error(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(ErrorLabeledModifier(), kMockLogger->LM());
// }

// // Log using NodeLogger with special header and default LM
// TEST(MockLogger, LogWithHeaderAndDefaultLM) {
//   auto n_logger = std::make_shared<NodeLogger>(kMockLogger);
//   // n_logger->SetLogHeader(kHeader);
//   n_logger->Info(kMessage);

//   n_logger->Debug(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(DebugLabeledModifier(), kMockLogger->LM());

//   n_logger->Warn(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(WarnLabeledModifier(), kMockLogger->LM());

//   n_logger->Info(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(InfoLabeledModifier(), kMockLogger->LM());

//   n_logger->Error(kMessage);
//   EXPECT_EQ(kHeaderMessage, kMockLogger->Msg());
//   ExpectEqLabeledModifier(ErrorLabeledModifier(), kMockLogger->LM());
// }

// // construct with special LM
// TEST(MockLogger, ConstructWithSpecialLM) {
//   auto n_logger = std::make_shared<NodeLogger>(kMockLogger, kNodeLm);

//   n_logger->Debug(kMessage);
//   ExpectEqLabeledModifier(kNodeLm.debug, kMockLogger->LM());

//   n_logger->Error(kMessage);
//   ExpectEqLabeledModifier(kNodeLm.error, kMockLogger->LM());

//   n_logger->Info(kMessage);
//   ExpectEqLabeledModifier(kNodeLm.info, kMockLogger->LM());

//   n_logger->Warn(kMessage);
//   ExpectEqLabeledModifier(kNodeLm.warn, kMockLogger->LM());
// }

// TEST(CreateFileAndConsoleLogger, CheckInitializedWriters) {
//   constexpr auto kName = "name";
//   constexpr auto kFilename = "name_logger.txt";
//   auto logger = core::utils::CreateFileAndConsoleLogger(kName, kFilename);

//   ConsoleBuffer c_buf;
//   logger.Debug(kMessage);
//   EXPECT_TRUE(
//     AssertFileAndConsole(kFilename, c_buf, DebugLabeledModifier(),
//     kMessage));
// }

TEST(CreateSystemNodeLogger, CheckInitializedWriters) {
  constexpr auto kFilename = "sys_logger.txt";
  const auto n_logger = core::utils::CreateSystemNodeLogger("");
  ConsoleBuffer c_buf;
  n_logger.Debug(kMessage);
  EXPECT_TRUE(AssertFileAndConsole(kFilename, c_buf, DebugLabeledModifier(),
                                   kMessage.data()));
}
