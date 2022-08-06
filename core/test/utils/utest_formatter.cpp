#include <gtest/gtest.h>

#include "core/utils/formatter.hpp"
#include "utest/utils.hpp"

using namespace core::utils;
using event_level_t = EventLevel::event_level_t;

// format a message with disabled Modifier
std::string format_func_with_modifier_disabled(const LabeledModifier& lm,
                                               const std::string& msg)
{
  std::stringstream ss;
  ss << "[" << DateTime().time_to_string() << "]";
  ss << "[" << event_level_to_string(lm.get_event_level()) << "]";
  ss << ": " << msg;
  return ss.str();
}

// format a message with enabled Modifier
std::string format_func_with_modifier_enabled(const LabeledModifier& lm,
                                              const std::string& msg)
{
  std::stringstream ss;
  ss << "[" << DateTime().time_to_string() << "]";
  ss << lm;
  ss << ": " << msg;
  return ss.str();
}

// test Null foramtter
TEST(NullFormatter, Format)
{
  std::string msg = "message";
  FormatterInterface* formatter = new NullFormatter();
  for (const auto& event : EVENTS)
  {
    EXPECT_EQ(msg, formatter->format(LabeledModifier(event), msg));
  }
}

// test Default foramtter's Format function constracted without disobliging the
// use of using modifier
TEST(DefaultFormater, FormatWithothModifier)
{
  std::string msg = "message";
  DefaultFormater formatter(false);
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    LabeledModifier lm(EVENTS[i], LABELS[i]);
    std::string expect = format_func_with_modifier_disabled(lm, msg);
    EXPECT_EQ(expect, formatter.format(lm, msg));
  }
}

// test Default foramtter's Format function constracted with enabling the use of
// modifier and time zone information
TEST(DefaultFormater, FormatWithModifier)
{
  std::string msg = "message";
  DefaultFormater formatter(true);
  for (size_t i = 0; i < EVENTS.size(); i++)
  {
    LabeledModifier lm(EVENTS[i], LABELS[i]);
    std::string expect = format_func_with_modifier_enabled(lm, msg);
    EXPECT_EQ(expect, formatter.format(lm, msg));
  }
}
