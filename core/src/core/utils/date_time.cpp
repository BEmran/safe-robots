// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/date_time.hpp"

#include <iostream>
#include <map>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace {
tm CreateTmStruct() {
  const auto raw_time = time(nullptr);
  tm time_struct{};
  localtime_r(&raw_time, &time_struct);
  return time_struct;
}

/**
 * @brief Format time_info_ struct to string by rapping strftime function
 *
 * @param format C string containing any combination of regular characters and
 * special format specifiers.
 * @param tm time structure contains a calendar date and time broken down into
 * its components.
 * @return std::string the formatted string
 */
std::string FormatTimeInfo(const char* format, const tm time_info) {
  constexpr auto kMaxSize = 50;
  std::string buffer(kMaxSize, ' ', std::allocator<char>());
  const auto actual_size =
    strftime(buffer.data(), buffer.size(), format, &time_info);
  if (actual_size > 0) {
    buffer.resize(actual_size);
  } else {
    std::wcerr << "FormatTimeInfo: Selected format exceeds string size"
               << std::endl;
  }
  return buffer;
}
}  // namespace

std::string GenerateFileName(const DateTime dt) {
  const tm time_struct = dt.GetTimeStruct();
  return FormatTimeInfo("%d-%b-%Y_%H-%M-%S", time_struct);
}

/*****************************************************************************/

DateTime::DateTime() : DateTime(CreateTmStruct()) {
}

DateTime::DateTime(const tm& time_struct) : time_info_(time_struct) {
}

std::string DateTime::DateToString() {
  return FormatTimeInfo("%d/%m/%Y", time_info_);
}

std::string DateTime::TimeToString() {
  return FormatTimeInfo("%H:%M:%S", time_info_);
}

std::string DateTime::Pretty() {
  return FormatTimeInfo("%a %d/%b/%Y %H:%M:%S", time_info_);
}

tm DateTime::GetTimeStruct() const {
  return time_info_;
}

std::string DateTime::AbbreviatedMonthName() const {
  return FormatTimeInfo("%b", time_info_);
}

std::string DateTime::AbbreviatedWeekdayName() const {
  return FormatTimeInfo("%a", time_info_);
}

}  // namespace core::utils
