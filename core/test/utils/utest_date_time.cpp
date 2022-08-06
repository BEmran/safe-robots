#include <core/utils/date_time.hpp>
#include <iomanip>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "utest/utils.hpp"

using namespace core::utils;

tm get_local_time()
{
  time_t now = time(nullptr);
  tm* local_tm = localtime(&now);
  return *local_tm;
}

long time_since_epoch_in_sec()
{
  using namespace std::chrono;
  auto now = system_clock::now();
  auto epoch = now.time_since_epoch();
  return duration_cast<seconds>(epoch).count();
}

testing::AssertionResult is_equal_date_time(const tm& expect, const tm& actual)
{
  return assert_eq_with_label<int>(expect.tm_year, actual.tm_year, "Year") &&
         assert_eq_with_label<int>(expect.tm_mon, actual.tm_mon, "Month") &&
         assert_eq_with_label<int>(expect.tm_mday, actual.tm_mday, "Day") &&
         assert_eq_with_label<int>(expect.tm_hour, actual.tm_hour, "Hour") &&
         assert_eq_with_label<int>(expect.tm_min, actual.tm_min, "Minute") &&
         assert_eq_with_label<int>(expect.tm_sec, actual.tm_sec, "Seconds");
}

// check time_in_seconds function
TEST(DateTime, time_in_seconds)
{
  auto expect = time_since_epoch_in_sec();
  EXPECT_EQ(expect, time_in_seconds());
}

// check time_in_seconds_string function
TEST(DateTime, time_in_seconds_string)
{
  auto expect = std::to_string(time_since_epoch_in_sec());
  EXPECT_EQ(expect, time_in_seconds_string());
}

// check generate_file_name function
TEST(DateTime, GenerateFileName)
{
  tm local_tm{};
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;
  local_tm.tm_mon = 5;
  local_tm.tm_mday = 6;
  const DateTime dt(local_tm);
  EXPECT_EQ("06-Jun-1904_01-02-03", generate_file_name(dt));
}

// check default construct
TEST(DateTime, DefaultConstruct)
{
  DateTime dt;
  EXPECT_TRUE(is_equal_date_time(get_local_time(), dt.get_time_struct()));
}

// check default construct
TEST(DateTime, CrateUsingATimeStruct)
{
  tm local_tm{};
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;
  local_tm.tm_mon = 5;
  local_tm.tm_yday = 6;
  const DateTime dt(local_tm);
  EXPECT_TRUE(is_equal_date_time(local_tm, dt.get_time_struct()));
}

// check constracting time format
TEST(DateTime, TimeToString)
{
  tm local_tm = get_local_time();
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;

  EXPECT_EQ("01:02:03", DateTime(local_tm).time_to_string());
}

// check constracting time format
TEST(DateTime, DateToString)
{
  tm local_tm = get_local_time();
  local_tm.tm_year = 1;  // years starts from 1900
  local_tm.tm_mon = 4;   // month starts from zeros
  local_tm.tm_mday = 3;

  EXPECT_EQ("03/05/1901", DateTime(local_tm).date_to_string());
}

// check abbreviated_month_name function
TEST(DateTime, abbreviated_month_name)
{
  tm local_tm = get_local_time();
  local_tm.tm_mon = 4;  // month starts from zeros

  EXPECT_EQ("May", DateTime(local_tm).abbreviated_month_name());
}

// check abbreviated_weekday_name function
TEST(DateTime, abbreviated_weekday_name)
{
  tm local_tm = get_local_time();
  local_tm.tm_wday = 2;  // days start from sunday

  EXPECT_EQ("Tue", DateTime(local_tm).abbreviated_weekday_name());
}

// check DateTime pretty function
TEST(DateTime, pretty)
{
  tm local_tm = get_local_time();
  local_tm.tm_hour = 1;
  local_tm.tm_min = 2;
  local_tm.tm_sec = 3;
  local_tm.tm_year = 4;  // years starts from 1900
  local_tm.tm_mon = 5;   // month starts from zeros
  local_tm.tm_mday = 1;
  local_tm.tm_wday = 1;  // days start from sunday
  EXPECT_EQ("Mon 01/Jun/1904 01:02:03", DateTime(local_tm).pretty());
}
