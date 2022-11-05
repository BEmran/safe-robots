// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <sys/stat.h>

#include <cstdlib>
#include <experimental/filesystem>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace fs = std::experimental::filesystem;

void CreateDirectories(std::string_view full_path) {
  fs::path path{full_path};
  try {
    fs::create_directories(path);
  } catch (fs::filesystem_error& e) {
    throw Exception(e.what());
  }
}

bool IsPathExists(std::string_view path) {
  struct stat buffer {};
  return stat(path.data(), &buffer) == 0;
}

bool IsProcessorAvailable() {
  return system(NULL) != 0;
}

std::optional<int> ExecuteSystemCommand(std::string_view cmd) {
  if (not IsProcessorAvailable()) {
    return {};
  }
  return system(cmd.data());
}

}  // namespace core::utils
