
#ifndef CORE_UTILS_SYSTEM_HPP
#define CORE_UTILS_SYSTEM_HPP

#include <string>

namespace core
{
namespace utils
{
/**
 * @brief Create a Directories by calling boost::filesystem::create_directory
 * for each element of full_path that does not exist.
 *
 * @param full_path full path of the directory
 * @throw CoreExcept if can not create directories
 */
void create_directories(const std::string& full_path);

/**
 * @brief Checks if the given path exists
 *
 * @param path full path
 * @return true if exists
 * @return false otherwise
 */
bool is_path_exists(const std::string& path);

}  // namespace utils
}  // namespace core

#endif  // CORE_UTILS_SYSTEM_HPP