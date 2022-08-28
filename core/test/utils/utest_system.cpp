// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <experimental/filesystem>

#include "core/utils/system.hpp"

namespace fs = std::experimental::filesystem;
using core::utils::CreateDirectories;
using core::utils::IsPathExists;

TEST(System, FileExist) {
  auto path = fs::current_path();
  EXPECT_TRUE(IsPathExists(path.string()));
}

TEST(System, CreateDirectories) {
  fs::path path = fs::temp_directory_path();
  path /= "utest/system/my/dir";
  std::string full_path = path.string();
  CreateDirectories(full_path);
  EXPECT_TRUE(IsPathExists(full_path));
  EXPECT_EQ(fs::remove_all(fs::temp_directory_path() / "utest"), 4);
}
