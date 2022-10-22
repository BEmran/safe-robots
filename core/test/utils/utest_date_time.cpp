// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <iomanip>
#include <sstream>
#include <string>

#include "core/utils/date_time.hpp"
#include "gtest/gtest.h"
#include "utest/utils.hpp"

using core::utils::DateTime;
using core::utils::GenerateFileName;
using core::utils::TimeInSeconds;
using core::utils::TimeInSecondsString;

tm GetLocalTime() {
  time_t now = time(nullptr);
  tm local_tm;
  localtime_r(&now, &local_tm);
  return local_tm;
}

int64_t TimeSinceEpochInSec() {
  auto now = std::chrono::system_clock::now();
  auto epoch = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
}

testing::AssertionResult IsEqualDateTime(const tm& expect, const tm& actual) {
  return AssertEqWithLabel(expect.tm_year, actual.tm_year, "Year") &&
         AssertEqWithLabel(expect.tm_mon, actual.tm_mon, "Month") &&
         AssertEqWithLabel(expect.tm_mday, actual.tm_mday, "Day") &&
         AssertEqWithLabel(expect.tm_hour, actual.tm_hour, "Hour") &&
         AssertEqWithLabel(expect.tm_min, actual.tm_min, "Minute") &&
         AssertEqWithLabel(expect.tm_sec, actual.tm_sec, "Seconds");
}

// check time_in_seconds function
TEST(DateTime, time_in_seconds) {
  const auto expect = TimeSinceEpochInSec();
  EXPECT_EQ(expect, TimeInSeconds());
}

// check CreateNodeUsingSystemLogger function
TEST(DateTime, CreateNodeUsingSystemLogger) {
  const auto expect = std::to_string(TimeSinceEpochInSec());
  EXPECT_EQ(expect, TimeInSecondsString());
}

// check generate_file_name function
TEST(DateTime, GenerateFileName) {
  tm local_tm{};
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;
  local_tm.tm_mon = 5;
  local_tm.tm_mday = 6;
  const DateTime dt(local_tm);
  EXPECT_EQ("06-Jun-1904_01-02-03", GenerateFileName(dt));
}

// check default construct
TEST(DateTime, DefaultConstruct) {
  DateTime dt;
  EXPECT_TRUE(IsEqualDateTime(GetLocalTime(), dt.GetTimeStruct()));
}

// check default construct
TEST(DateTime, CrateUsingATimeStruct) {
  tm local_tm{};
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;
  local_tm.tm_mon = 5;
  local_tm.tm_yday = 6;
  const DateTime dt(local_tm);
  EXPECT_TRUE(IsEqualDateTime(local_tm, dt.GetTimeStruct()));
}

// check constructing time format
TEST(DateTime, TimeToString) {
  tm local_tm = GetLocalTime();
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;

  EXPECT_EQ("01:02:03", DateTime(local_tm).TimeToString());
}

// check constructing time format
TEST(DateTime, DateToString) {
  tm local_tm = GetLocalTime();
  local_tm.tm_year = 1;  // years starts from 1900
  local_tm.tm_mon = 4;   // month starts from zeros
  local_tm.tm_mday = 3;

  EXPECT_EQ("03/05/1901", DateTime(local_tm).DateToString());
}

// check abbreviated_month_name function
TEST(DateTime, abbreviated_month_name) {
  tm local_tm = GetLocalTime();
  local_tm.tm_mon = 4;  // month starts from zeros

  EXPECT_EQ("May", DateTime(local_tm).AbbreviatedMonthName());
}

// check abbreviated_weekday_name function
TEST(DateTime, abbreviated_weekday_name) {
  tm local_tm = GetLocalTime();
  local_tm.tm_wday = 2;  // days start from sunday

  EXPECT_EQ("Tue", DateTime(local_tm).AbbreviatedWeekdayName());
}

// check DateTime pretty function
TEST(DateTime, pretty) {
  tm local_tm = GetLocalTime();
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;  // years starts from 1900
  local_tm.tm_mon = 5;   // month starts from zeros
  local_tm.tm_mday = 1;
  local_tm.tm_wday = 1;  // days start from sunday
  EXPECT_EQ("Mon 01/Jun/1904 01:02:03", DateTime(local_tm).Pretty());
}
