#ifndef CORE_UTILS_TERMINAL_HPP
#define CORE_UTILS_TERMINAL_HPP

#include <list>
#include <ostream>
#include <vector>

// Bash colors and formatting (ANSI/VT100 Control sequences)
// https://misc.flogisoft.com/bash/tip_colors_and_formatting
namespace core::utils {
namespace terminal {
// foreground color
enum FG {
  FG_BLACK = 30,          // black
  FG_RED = 31,            // red
  FG_GREEN = 32,          // green
  FG_YELLOW = 33,         // yellow
  FG_BLUE = 34,           // blue
  FG_MAGENTA = 35,        // magenta
  FG_CYAN = 36,           // cyan
  FG_LIGHT_GRAY = 37,     // light gray
  FG_DEFAULT = 39,        // default
  FG_DARK_GRAY = 90,      // dark gray
  FG_LIGHT_RED = 91,      // light red
  FG_LIGHT_GREEN = 92,    // light green
  FG_LIGHT_YELLOW = 93,   // light yellow
  FG_LIGHT_BLUE = 94,     // light blue
  FG_LIGHT_MAGENTA = 95,  // light magenta
  FG_LIGHT_CYAN = 96,     // light cyan
  FG_WHITE = 97,          // white
};

// background color
enum BG {
  BG_RED = 41,             // red
  BG_GREEN = 42,           // green
  BG_YELLOW = 43,          // yellow
  BG_BLUE = 44,            // blue
  BG_MAGENTA = 45,         // magenta
  BG_CYAN = 46,            // cyan
  BG_LIGHT_GRAY = 47,      // light gray
  BG_DEFAULT = 49,         // default
  BG_DARK_GRAY = 100,      // dark gray
  BG_LIGHT_RED = 101,      // light red
  BG_LIGHT_GREEN = 102,    // light green
  BG_LIGHT_YELLOW = 103,   // light yellow
  BG_LIGHT_BLUE = 104,     // light blue
  BG_LIGHT_MAGENTA = 105,  // light magenta
  BG_LIGHT_CYAN = 106,     // light cyan
  BG_WHITE = 107,          // white
};

// font format
enum FMT {
  FMT_DEFAULT = 0,    // normal text
  FMT_BOLD = 1,       // bold
  FMT_DIM = 2,        // dim
  FMT_UNDERLINE = 4,  // underline
  FMT_BLINK = 5,      // blinking
  FMT_REVERSE = 7,    // reverse coloring
  FMT_HIDDEN = 8,     // hidden
};
}  // namespace terminal

/**
 * @brief Used to format the data printed at a terminal
 *
 */
class Modifier {
 public:
  /**
   * @brief Construct a new Modifier object
   *
   * @param fg  foreground color
   * @param bg  background color
   * @param fmt font format
   */
  Modifier(const terminal::FG fg, const terminal::BG bg,
           const terminal::FMT fmt);
  std::string ToString() const;

 private:
  terminal::FG fg_;    // foreground color
  terminal::BG bg_;    // background color
  terminal::FMT fmt_;  // font format
};

/**
 *
 * @brief overloading the ostream operator
 *
 * @param os ostream object
 * @param mod modifier to be used
 * @return std::ostream& updated ostream
 */
std::ostream& operator<<(std::ostream& os, const Modifier& mod);

/* Create Default modifiers*/
Modifier DefaultModifier();
Modifier DebugModifier();
Modifier ErrorModifier();
Modifier InfoModifier();
Modifier WarnModifier();

}  // namespace core::utils

#endif  // CORE_UTILS_TERMINAL_HPP
