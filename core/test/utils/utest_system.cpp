#include <gtest/gtest.h>

#include <experimental/filesystem>

#include "core/utils/system.hpp"

namespace fs = std::experimental::filesystem;
using namespace core::utils;

TEST(System, FileExist)
{
  auto path = fs::current_path();
  EXPECT_TRUE(is_path_exists(path.string()));
}

TEST(System, CreateDirectories)
{
  fs::path path = fs::temp_directory_path();
  path /= "utest/system/my/dir";
  std::string full_path = path.string();
  create_directories(full_path);
  EXPECT_TRUE(is_path_exists(full_path));
  EXPECT_EQ(fs::remove_all(fs::temp_directory_path() / "utest"), 4);
}