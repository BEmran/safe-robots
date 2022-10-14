// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <sstream>

#include "core/utils/writer.hpp"
#include "core/utils/writer_file.hpp"
#include "utest/utils.hpp"

using core::utils::FileWriter;
using core::utils::Writer;
using namespace std::literals;  // s

constexpr std::string_view msg1{"msg 1"};
constexpr std::string_view msg2{"msg 2"};

TEST(Writer, LogThroughWriteFunc) {
  std::stringstream ss;
  Writer w(ss);

  w.Write(msg1);
  EXPECT_EQ(msg1.data(), ss.str());

  ss.str("");  // clear stringstream

  w.Write(msg2);
  EXPECT_EQ(msg2.data(), ss.str());
}

TEST(Writer, LogThroughStream) {
  std::stringstream ss;
  Writer w(ss);

  w << msg1;
  EXPECT_EQ(msg1, ss.str());

  ss.str("");  // clear stringstream

  w << msg2;
  EXPECT_EQ(msg2, ss.str());
}

TEST(FileWriter, LogThroughWriteFunc) {
  const std::string file_name = "utest_file_writer_log.txt";
  FileWriter writer(file_name);

  writer.Write(msg1.data() + "\n"s);
  writer.Write(msg2.data() + "\n"s);
  auto logged_data = ReadAllLinesFromFile(file_name);

  ASSERT_EQ(2, logged_data.size());
  EXPECT_EQ(msg1, logged_data.front());
  logged_data.pop_front();
  EXPECT_EQ(msg2, logged_data.front());
}

TEST(FileWriter, LogThroughStream) {
  const std::string file_name = "utest_file_writer_log_stream.txt";
  FileWriter writer(file_name);

  writer << msg1 << std::endl;
  writer << msg2 << std::endl;
  auto logged_data = ReadAllLinesFromFile(file_name);

  ASSERT_EQ(2, logged_data.size());
  EXPECT_EQ(msg1, logged_data.front());
  logged_data.pop_front();
  EXPECT_EQ(msg2, logged_data.front());
}
