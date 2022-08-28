// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/writter_console.hpp"
#include "core/utils/writter_file.hpp"
#include "utest/utils.hpp"

using core::utils::ConsoleWritter;
using core::utils::FileWritter;

TEST(WritterConsole, LogOnceNoName) {
  ConsoleBuffer buf;
  const std::string data_to_log = "some info";
  ConsoleWritter().dump(data_to_log);
  auto logged_data = buf.RestoreCoutBuffer();
  ASSERT_EQ(1, logged_data.size());
  EXPECT_EQ(data_to_log, logged_data.front());
}

TEST(WritterFile, LogOnce) {
  const std::string file_name = "utest_writter_file_single.txt";
  FileWritter writter(file_name);
  const std::string data_to_log = "some info 2";
  writter.dump(data_to_log);
  auto logged_data = ReadAllLinesFromFile(file_name);
  ASSERT_EQ(1, logged_data.size());
  EXPECT_EQ(data_to_log, logged_data.front());
}
