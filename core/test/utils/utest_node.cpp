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

// check construct Node with Name
// TEST(Node, ConstructWithName) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const Node node("node name", mock_logger);

//   EXPECT_EQ(node.GetName(), "node name");
//   EXPECT_EQ(node.GetLogger(), mock_logger);
// }

// // check construct Node with Name and Logger
// TEST(Node, ConstructWithNameAndLoggerAndModifier) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const NodeLabeledModifiers node_lm;
//   const Node node("node name", mock_logger, node_lm);

//   EXPECT_EQ(node.GetName(), "node name");
//   EXPECT_EQ(node.GetLogger(), mock_logger);
// }

// // check Node Log function using MockLogger
// TEST(Node, LogWithArbitraryLabeledModifier) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const Node node("node name", mock_logger);
//   const core::utils::LabeledModifier lm(EventLevel::EL_INFO, "tmp",
//                                         DefaultModifier());
//   node.Log(lm, "message");
//   ExpectEqLabeledModifier(mock_logger->LM(), lm);
// }

// // check Node log_debug function to see if it calls Logger.Log function with
// the
// // passed message and Debug LabeledModifier using MockLogger
// TEST(Node, log_debug) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const NodeLabeledModifiers node_lm;
//   const Node node("name", mock_logger, node_lm);
//   node.LogDebug("message");
//   ExpectEqLabeledModifier(mock_logger->LM(), node_lm.debug);
//   EXPECT_EQ(mock_logger->Msg(), "[name]: message");
// }

// // check Node log_error function to see if it calls Logger.Log function with
// the
// // passed message and Error LabeledModifier using MockLogger
// TEST(Node, log_error) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const NodeLabeledModifiers node_lm;
//   const Node node("name", mock_logger, node_lm);
//   node.LogError("message");
//   ExpectEqLabeledModifier(mock_logger->LM(), node_lm.error);
//   EXPECT_EQ(mock_logger->Msg(), "[name]: message");
// }

// // check Node log_info function to see if it calls Logger.Log function with
// the
// // passed message and Info LabeledModifier using MockLogger
// TEST(Node, log_info) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const NodeLabeledModifiers node_lm;
//   const Node node("name", mock_logger, node_lm);
//   node.LogInfo("message");
//   ExpectEqLabeledModifier(mock_logger->LM(), node_lm.info);
//   EXPECT_EQ(mock_logger->Msg(), "[name]: message");
// }

// // check Node log_warn function to see if it calls Logger.Log function with
// the
// // passed message and Warn LabeledModifier using MockLogger
// TEST(Node, log_warn) {
//   auto mock_logger = std::make_shared<MockLogger>();
//   const NodeLabeledModifiers node_lm;
//   Node node("name", mock_logger, node_lm);
//   node.LogWarn("message");
//   ExpectEqLabeledModifier(mock_logger->LM(), node_lm.warn);
//   EXPECT_EQ(mock_logger->Msg(), "[name]: message");
// }
