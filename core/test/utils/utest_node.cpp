#include <gtest/gtest.h>

#include "core/utils/logger.hpp"
#include "core/utils/node.hpp"
#include "utils.hpp"

using namespace core::utils;

class MockLogger : public Logger
{
 public:
  MockLogger() : Logger("filename"), lm_(LabeledModifier(EventLevel::EL_DEBUG))
  {
  }
  void Log(const LabeledModifier& lm, const std::string& msg) override
  {
    lm_ = lm;
    msg_ = msg;
  }

  [[nodiscard]] std::string Msg() const
  {
    return msg_;
  }

  [[nodiscard]] LabeledModifier LM() const
  {
    return lm_;
  }

 private:
  LabeledModifier lm_;
  std::string msg_;
};

// check creating NodeLabel Modifier
TEST(NodeLabeledModifiers, Construct)
{
  const LabeledModifier lm_debug(EventLevel::EL_DEBUG, DebugModifier());
  const LabeledModifier lm_error(EventLevel::EL_ERROR, ErrorModifier());
  const LabeledModifier lm_info(EventLevel::EL_INFO, InfoModifier());
  const LabeledModifier lm_warn(EventLevel::EL_WARN, WarnModifier());
  const NodeLabeledModifiers node_lm(lm_debug, lm_error, lm_info, lm_warn);

  expect_eq_labeled_modifier(node_lm.debug, lm_debug);
  expect_eq_labeled_modifier(node_lm.error, lm_error);
  expect_eq_labeled_modifier(node_lm.info, lm_info);
  expect_eq_labeled_modifier(node_lm.warn, lm_warn);
}

// check creating Default NodeLabel Modifier using
// CreateDefaultNodeLabelModifier function
TEST(NodeLabeledModifiers, DefaultConstruct)
{
  const NodeLabeledModifiers node_lm;
  expect_eq_labeled_modifier(node_lm.debug, DebugLabeledModifier());
  expect_eq_labeled_modifier(node_lm.error, ErrorLabeledModifier());
  expect_eq_labeled_modifier(node_lm.info, InfoLabeledModifier());
  expect_eq_labeled_modifier(node_lm.warn, WarnLabeledModifier());
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLogger)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const Node node("node name", mock_logger);

  EXPECT_EQ(node.GetName(), "node name");
  EXPECT_EQ(node.GetLogger(), mock_logger);
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLoggerAndModiffier)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("node name", mock_logger, node_lm);

  EXPECT_EQ(node.GetName(), "node name");
  EXPECT_EQ(node.GetLogger(), mock_logger);
}

// check Node Log function using MockLogger
TEST(Node, LogWithArbitraryLabledModiffier)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const Node node("node name", mock_logger);
  const core::utils::LabeledModifier lm(EventLevel::EL_INFO, "tmp",
                                        DefaultModifier());
  node.Log(lm, "message");
  expect_eq_labeled_modifier(mock_logger->LM(), lm);
}

// check Node log_debug function to see if it calls Logger.Log function with the
// passed message and Debug LabeledModifer using MockLogger
TEST(Node, log_debug)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogDebug("message");
  expect_eq_labeled_modifier(mock_logger->LM(), node_lm.debug);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_error function to see if it calls Logger.Log function with the
// passed message and Error LabeledModifer using MockLogger
TEST(Node, log_error)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogError("message");
  expect_eq_labeled_modifier(mock_logger->LM(), node_lm.error);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_info function to see if it calls Logger.Log function with the
// passed message and Info LabeledModifer using MockLogger
TEST(Node, log_info)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  const Node node("name", mock_logger, node_lm);
  node.LogInfo("message");
  expect_eq_labeled_modifier(mock_logger->LM(), node_lm.info);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}

// check Node log_warn function to see if it calls Logger.Log function with the
// passed message and Warn LabeledModifer using MockLogger
TEST(Node, log_warn)
{
  auto mock_logger = std::make_shared<MockLogger>();
  const NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.LogWarn("message");
  expect_eq_labeled_modifier(mock_logger->LM(), node_lm.warn);
  EXPECT_EQ(mock_logger->Msg(), "[name]: message");
}
