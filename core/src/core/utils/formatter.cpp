
#include "core/utils/formatter.hpp"

#include <sstream>

namespace core
{
namespace utils
{
std::string NullFormatter::format(const LabeledModifier& lm,
                                  const std::string& msg) const
{
  (void)lm;
  return msg;
}

DefaultFormater::DefaultFormater(const bool use_modifier)
  : use_modifier_(use_modifier)
{
}

std::string DefaultFormater::format(const LabeledModifier& lm,
                                    const std::string& msg) const
{
  std::stringstream ss;
  ss << "[" << DateTime().time_to_string() << "]";
  ss << add_labeled_modifier(lm);
  ss << ": " << msg;
  return ss.str();
}

std::string
DefaultFormater::add_labeled_modifier(const LabeledModifier& lm) const
{
  std::stringstream ss;
  if (use_modifier_)
  {
    ss << lm;
  }
  else
  {
    ss << "[" << lm.get_label() << "]";
  }
  return ss.str();
}
}  // namespace utils
}  // namespace core
