// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <sys/stat.h>

#include <experimental/filesystem>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace fs = std::experimental::filesystem;

void CreateDirectories(const std::string& full_path) {
  fs::path path{full_path};
  try {
    fs::create_directories(path);
  } catch (fs::filesystem_error& e) {
    throw Exception(e.what());
  }
}

bool IsPathExists(const std::string& path) {
  struct stat buffer {};
  return stat(path.c_str(), &buffer) == 0;
}

}  // namespace core::utils
