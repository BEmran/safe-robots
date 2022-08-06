#include <gtest/gtest.h>

#include "core/utils/logger.hpp"
#include "core/utils/node.hpp"
#include "utest/utils.hpp"

using namespace core::utils;

class MockLogger : public Logger
{
 public:
  MockLogger()
    : Logger("filename")
    , m_lm(LabeledModifier(EventLevel::event_level_t::EL_DEBUG))
    , m_msg{}
  {
  }
  void log(const LabeledModifier& lm, const std::string& msg) override
  {
    m_lm = lm;
    m_msg = msg;
  }
  LabeledModifier m_lm;
  std::string m_msg;
};

// check creating NodeLabel Modifier
TEST(NodeLabeledModifiers, Construct)
{
  LabeledModifier lm(EventLevel::event_level_t::EL_DEBUG, debug_modifier());
  NodeLabeledModifiers node_lm(lm, lm, lm, lm);

  expect_eq_labeled_modifier(node_lm.debug, lm);
  expect_eq_labeled_modifier(node_lm.error, lm);
  expect_eq_labeled_modifier(node_lm.info, lm);
  expect_eq_labeled_modifier(node_lm.warn, lm);
}

// check creating Default NodeLabel Modifier using
// CreateDefaultNodeLabelModifier function
TEST(NodeLabeledModifiers, DefaultConstruct)
{
  NodeLabeledModifiers node_lm;
  expect_eq_labeled_modifier(node_lm.debug, debug_labeled_modifier());
  expect_eq_labeled_modifier(node_lm.error, error_labeled_modifier());
  expect_eq_labeled_modifier(node_lm.info, info_labeled_modifier());
  expect_eq_labeled_modifier(node_lm.warn, warn_labeled_modifier());
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLogger)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("node name", mock_logger);

  EXPECT_EQ(node.get_name(), "node name");
  EXPECT_EQ(node.get_logger(), mock_logger);
}

// check construct Node with Name and Logger
TEST(Node, ConstructWithNameAndLoggerAndModiffier)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("node name", mock_logger, node_lm);

  EXPECT_EQ(node.get_name(), "node name");
  EXPECT_EQ(node.get_logger(), mock_logger);
}

// check Node Log function using MockLogger
TEST(Node, LogWithArbitraryLabledModiffier)
{
  auto mock_logger = std::make_shared<MockLogger>();
  Node node("node name", mock_logger);
  core::utils::LabeledModifier lm(event_level_t::EL_INFO, "tmp", default_modifier());
  node.log(lm, "message");
  expect_eq_labeled_modifier(mock_logger->m_lm, lm);
}

// check Node log_debug function to see if it calls Logger.Log function with the
// passed message and Debug LabeledModifer using MockLogger
TEST(Node, log_debug)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.log_debug("message");
  expect_eq_labeled_modifier(mock_logger->m_lm, node_lm.debug);
  EXPECT_EQ(mock_logger->m_msg, "[name]: message");
}

// check Node log_error function to see if it calls Logger.Log function with the
// passed message and Error LabeledModifer using MockLogger
TEST(Node, log_error)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.log_error("message");
  expect_eq_labeled_modifier(mock_logger->m_lm, node_lm.error);
  EXPECT_EQ(mock_logger->m_msg, "[name]: message");
}

// check Node log_info function to see if it calls Logger.Log function with the
// passed message and Info LabeledModifer using MockLogger
TEST(Node, log_info)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.log_info("message");
  expect_eq_labeled_modifier(mock_logger->m_lm, node_lm.info);
  EXPECT_EQ(mock_logger->m_msg, "[name]: message");
}

// check Node log_warn function to see if it calls Logger.Log function with the
// passed message and Warn LabeledModifer using MockLogger
TEST(Node, log_warn)
{
  auto mock_logger = std::make_shared<MockLogger>();
  NodeLabeledModifiers node_lm;
  Node node("name", mock_logger, node_lm);
  node.log_warn("message");
  expect_eq_labeled_modifier(mock_logger->m_lm, node_lm.warn);
  EXPECT_EQ(mock_logger->m_msg, "[name]: message");
}
