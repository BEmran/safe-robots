#include "core/utils/writter_console.hpp"

#include <iostream>

namespace
{
/**
 * @brief the actual function that logs string to console
 *
 * @param str string to be logged
 */
void dump_to_console(const std::string& str)
{
  std::cout << str << std::endl;
}
}  // namespace


namespace core
{
namespace utils
{
ConsoleWritter::ConsoleWritter()
{
}

ConsoleWritter::~ConsoleWritter()
{
}

void ConsoleWritter::dump(const std::string& str)
{
  dump_to_console(str);
}

}  // namespace utils
}  // namespace core