// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/logger.hpp"
#include "core/utils/node.hpp"
#include "utest/utils.hpp"

using core::utils::DebugLabeledModifier;
using core::utils::DebugModifier;
using core::utils::DefaultModifier;
using core::utils::ErrorLabeledModifier;
using core::utils::ErrorModifier;
using core::utils::InfoLabeledModifier;
using core::utils::InfoModifier;
using core::utils::LabeledModifier;
using core::utils::Logger;
using core::utils::Node;
using core::utils::NodeLabeledModifiers;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

class MockLogger : public Logger {
 public:
  MockLogger()
    : Logger("filename"), lm_(LabeledModifier(EventLevel::EL_DEBUG)) {
  }
  void Log(const LabeledModifier& lm, const std::string& msg) override {
    lm_ = lm;
    msg_ = msg;
  }

  [[nodiscard]] std::string Msg() const {
    return msg_;
  }

  [[nodiscard]] LabeledModifier LM() const {
    return lm_;
  }

 private:
  LabeledModifier lm_;
  std::string msg_;
};

// check creating NodeLabel Modifier
TEST(NodeLabeledModifiers, Construct) {
  const LabeledModifier lm_debug(EventLevel::EL_DEBUG, DebugModifier());
  const LabeledModifier lm_error(EventLevel::EL_ERROR, ErrorModifier());
  const LabeledModifier lm_info(EventLevel::EL_INFO, InfoModifier());
  const LabeledModifier lm_warn(EventLevel::EL_WARN, WarnModifier());
  const NodeLabeledModifiers node_lm(lm_debug, lm_error, lm_info, lm_warn);

  ExpectEqLabeledModifier(node_lm.debug, lm_debug);
  ExpectEqLabeledModifier(node_lm.error, lm_error);
  ExpectEqLabeledModifier(node_lm.info, lm_info);
  ExpectEqLabeledModifier(node_lm.warn, lm_warn);
}

// check creating Default NodeLabel Modifier using
// CreateDefaultNodeLabelModifier function
TEST(NodeLabeledModifiers, DefaultConstruct) {
  const NodeLabeledModifiers node_lm;
  ExpectEqLabeledModifier(node_lm.debug, DebugLabeledModifier());
  ExpectEqLabeledModifier(node_lm.error, ErrorLabeledModifier());
  ExpectEqLabeledModifier(node_lm.info, InfoLabeledModifier());
  ExpectEqLabeledModifier(node_lm.warn, WarnLabeledModifier());
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLogger) {
  auto mock_logger = std::make_shared<MockLogger>();
  const Node node("node name", mock_logger);

  EXPECT_EQ(node.GetName(), "node name");
  EXPECT_EQ(node.GetLogger(), mock_logger);
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLoggerAndModifier) {
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("node name", mock_logger, node_lm);

  EXPECT_EQ(node.GetName(), "node name");
  EXPECT_EQ(node.GetLogger(), mock_logger);
}

// check Node Log function using MockLogger
TEST(Node, LogWithArbitraryLabeledModifier) {
  auto mock_logger = std::make_shared<MockLogger>();
  const Node node("node name", mock_logger);
  const core::utils::LabeledModifier lm(EventLevel::EL_INFO, "tmp",
                                        DefaultModifier());
  node.Log(lm, "message");
  ExpectEqLabeledModifier(mock_logger->LM(), lm);
}

// check Node log_debug function to see if it calls Logger.Log function with the
// passed message and Debug LabeledModifier using MockLogger
TEST(Node, log_debug) {
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogDebug("message");
  ExpectEqLabeledModifier(mock_logger->LM(), node_lm.debug);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_error function to see if it calls Logger.Log function with the
// passed message and Error LabeledModifier using MockLogger
TEST(Node, log_error) {
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogError("message");
  ExpectEqLabeledModifier(mock_logger->LM(), node_lm.error);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_info function to see if it calls Logger.Log function with the
// passed message and Info LabeledModifier using MockLogger
TEST(Node, log_info) {
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogInfo("message");
  ExpectEqLabeledModifier(mock_logger->LM(), node_lm.info);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_warn function to see if it calls Logger.Log function with the
// passed message and Warn LabeledModifier using MockLogger
TEST(Node, log_warn) {
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.LogWarn("message");
  ExpectEqLabeledModifier(mock_logger->LM(), node_lm.warn);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}
