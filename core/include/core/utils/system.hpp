// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SYSTEM_HPP_
#define CORE_UTILS_SYSTEM_HPP_

#include <optional>
#include <string_view>

namespace core::utils {
/**
 * @brief Create a Directories by calling boost::filesystem::create_directory
 * for each element of path that does not exist.
 *
 * @param path full path of the directory
 * @throw CoreExcept if can not create directories
 */
void CreateDirectories(std::string_view path);

/**
 * @brief Checks if the given path exists
 *
 * @param path full path
 * @return true if exists
 * @return false otherwise
 */
bool IsPathExists(std::string_view path);

/**
 * @brief Checking if processor is available
 *
 * @return true in case a command processor is available
 * @return false otherwise
 */
bool IsProcessorAvailable();

/**
 * @brief Invokes the command processor if available to execute a command
 *
 * @param cmd command to be executed
 * @return int the status code returned by the called command
 */
std::optional<int> ExecuteSystemCommand(std::string_view cmd);

}  // namespace core::utils

#endif  // CORE_UTILS_SYSTEM_HPP_
