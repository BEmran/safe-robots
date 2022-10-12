// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_MODIFIER_HPP_
#define CORE_UTILS_MODIFIER_HPP_

#include <ostream>
#include <string>
#include <vector>

// Bash colors and formatting (ANSI/VT100 Control sequences)
// https://misc.flogisoft.com/bash/tip_colors_and_formatting
namespace core::utils {
// foreground color
namespace FG {
constexpr int BLACK = 30;          // black
constexpr int RED = 31;            // red
constexpr int GREEN = 32;          // green
constexpr int YELLOW = 33;         // yellow
constexpr int BLUE = 34;           // blue
constexpr int MAGENTA = 35;        // magenta
constexpr int CYAN = 36;           // cyan
constexpr int LIGHT_GRAY = 37;     // light gray
constexpr int DEFAULT = 39;        // default
constexpr int DARK_GRAY = 90;      // dark gray
constexpr int LIGHT_RED = 91;      // light red
constexpr int LIGHT_GREEN = 92;    // light green
constexpr int LIGHT_YELLOW = 93;   // light yellow
constexpr int LIGHT_BLUE = 94;     // light blue
constexpr int LIGHT_MAGENTA = 95;  // light magenta
constexpr int LIGHT_CYAN = 96;     // light cyan
constexpr int WHITE = 97;          // white
};                                 // namespace FG

// background color
namespace BG {
constexpr int RED = 41;             // red
constexpr int GREEN = 42;           // green
constexpr int YELLOW = 43;          // yellow
constexpr int BLUE = 44;            // blue
constexpr int MAGENTA = 45;         // magenta
constexpr int CYAN = 46;            // cyan
constexpr int LIGHT_GRAY = 47;      // light gray
constexpr int DEFAULT = 49;         // default
constexpr int DARK_GRAY = 100;      // dark gray
constexpr int LIGHT_RED = 101;      // light red
constexpr int LIGHT_GREEN = 102;    // light green
constexpr int LIGHT_YELLOW = 103;   // light yellow
constexpr int LIGHT_BLUE = 104;     // light blue
constexpr int LIGHT_MAGENTA = 105;  // light magenta
constexpr int LIGHT_CYAN = 106;     // light cyan
constexpr int WHITE = 107;          // white
};                                  // namespace BG

// font format
namespace FMT {
constexpr int DEFAULT = 0;    // normal text
constexpr int BOLD = 1;       // bold
constexpr int DIM = 2;        // dim
constexpr int UNDERLINE = 4;  // underline
constexpr int BLINK = 5;      // blinking
constexpr int REVERSE = 7;    // reverse coloring
constexpr int HIDDEN = 8;     // hidden
};                            // namespace FMT

/**
 * @brief Used to format the data printed at a terminal
 *
 */
class Modifier {
 public:
  /**
   * @brief Construct a new Modifier object
   *
   * @param options  vector holds the desired options
   */
  Modifier(const std::vector<int>& options = {});
  std::string ToString() const;

 private:
  const std::string options_string_;
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
Modifier FatalModifier();
Modifier InfoModifier();
Modifier WarnModifier();

}  // namespace core::utils

#endif  // CORE_UTILS_MODIFIER_HPP_
