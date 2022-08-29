// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/writer_console.hpp"
#include "core/utils/writer_file.hpp"
#include "utest/utils.hpp"

using core::utils::ConsoleWriter;
using core::utils::FileWriter;

TEST(WriterConsole, LogOnceNoName) {
  ConsoleBuffer buf;
  const std::string data_to_log = "some info";
  ConsoleWriter().Dump(data_to_log);
  auto logged_data = buf.RestoreCoutBuffer();
  ASSERT_EQ(1, logged_data.size());
  EXPECT_EQ(data_to_log, logged_data.front());
}

TEST(WriterFile, LogOnce) {
  const std::string file_name = "utest_writer_file_single.txt";
  FileWriter writer(file_name);
  const std::string data_to_log = "some info 2";
  writer.Dump(data_to_log);
  auto logged_data = ReadAllLinesFromFile(file_name);
  ASSERT_EQ(1, logged_data.size());
  EXPECT_EQ(data_to_log, logged_data.front());
}
