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
using core::utils::LoggerLabeledModifiers;
using core::utils::Node;
using core::utils::WarnLabeledModifier;
using core::utils::WarnModifier;

constexpr const char* kNodeName = "node";

// construct Node with Name
TEST(Node, ConstructWithName) {
  const Node node(kNodeName);
  EXPECT_EQ(node.GetName(), kNodeName);
}

// // construct Node with Name and logger
// TEST(Node, ConstructWithNameAndLogger) {
//   auto node_logger = core::utils::CreateNodeLogger("name");
//   const Node node(kNodeName, node_logger);
//   EXPECT_EQ(node.GetLogger(), node_logger);
// }

// construct system Node
TEST(CreateSystemNode, ConstructNode) {
  auto node = core::utils::CreateSystemNode(kNodeName);
  EXPECT_EQ(node.GetName(), kNodeName);
}
