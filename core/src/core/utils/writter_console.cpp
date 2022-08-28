#include "core/utils/writter_console.hpp"

#include <iostream>

namespace {
/**
 * @brief the actual function that logs string to console
 *
 * @param str string to be logged
 */
void DumpToConsole(const std::string& str) {
  std::cout << str << std::endl;
}
}  // namespace

namespace core::utils {
void ConsoleWritter::dump(const std::string& str) {
  DumpToConsole(str);
}

}  // namespace core::utils