#include "core/utils/terminal.hpp"

namespace core
{
namespace utils
{

using FG = terminal::FG;
using BG = terminal::BG;
using FMT = terminal::FMT;

namespace
{
std::string single_option_to_string(int option)
{
  return "\x1B[" + std::to_string(option) + "m";
}
}  // namespace

Modifier::Modifier(const FG fg, const BG bg, const FMT fmt)
  : fg_(fg), bg_(bg), fmt_(fmt)
{
}

std::string Modifier::to_string() const
{
  return single_option_to_string(fmt_)    // change baground
         + single_option_to_string(fg_)   // change font format
         + single_option_to_string(bg_);  // change foreground
}

std::ostream& operator<<(std::ostream& os, const Modifier& mod)
{
  os << mod.to_string();
  return os;
}

Modifier default_modifier()
{
  return Modifier(FG::FG_DEFAULT, BG::BG_DEFAULT, FMT::FMT_DEFAULT);
}

Modifier debug_modifier()
{
  return Modifier(FG::FG_LIGHT_BLUE, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier error_modifier()
{
  return Modifier(FG::FG_LIGHT_RED, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier info_modifier()
{
  return Modifier(FG::FG_LIGHT_CYAN, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

Modifier warn_modifier()
{
  return Modifier(FG::FG_LIGHT_YELLOW, BG::BG_DEFAULT, FMT::FMT_BOLD);
}

}  // namespace utils
}  // namespace core
