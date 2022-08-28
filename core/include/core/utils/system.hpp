// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_UTILS_SYSTEM_HPP_
#define CORE_INCLUDE_CORE_UTILS_SYSTEM_HPP_

#include <string>

namespace core::utils {
/**
 * @brief Create a Directories by calling boost::filesystem::create_directory
 * for each element of full_path that does not exist.
 *
 * @param full_path full path of the directory
 * @throw CoreExcept if can not create directories
 */
void CreateDirectories(const std::string& full_path);

/**
 * @brief Checks if the given path exists
 *
 * @param path full path
 * @return true if exists
 * @return false otherwise
 */
bool IsPathExists(const std::string& path);

}  // namespace core::utils

#endif  // CORE_INCLUDE_CORE_UTILS_SYSTEM_HPP_
