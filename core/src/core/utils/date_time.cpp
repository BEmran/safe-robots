#include "core/utils/date_time.hpp"

#include <iostream>
#include <map>

#include "core/utils/exception.hpp"

namespace core::utils
{
namespace
{
tm CraeteTmStruct()
{
  const auto raw_time = time(nullptr);
  return *localtime(&raw_time);
}
}  // namespace

int64_t TimeInSeconds()
{
  const auto now = std::chrono::system_clock::now();
  const auto epoch = now.time_since_epoch();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
  return sec;
}

std::string TimeInSecondsString()
{
  char buff[15];
  sprintf(buff, "%lld", TimeInSeconds());
  return buff;
}

std::string GenerateFileName(const DateTime dt)
{
  char buffer[21];
  tm time_struct = dt.GetTimeStruct();
  strftime(buffer, 21, "%d-%b-%Y_%H-%M-%S", &time_struct);
  return buffer;
}

/*****************************************************************************/

DateTime::DateTime() : DateTime(CraeteTmStruct())
{
}

DateTime::DateTime(const tm& time_struct) : time_info_(time_struct)
{
}

std::string DateTime::DateToString()
{
  char buffer[11];
  strftime(buffer, 11, "%d/%m/%Y", &time_info_);
  return buffer;
}

std::string DateTime::TimeToString()
{
  char buffer[9];
  strftime(buffer, 9, "%H:%M:%S", &time_info_);
  return buffer;
}

std::string DateTime::Pretty()
{
  char buffer[31];
  strftime(buffer, 31, "%a %d/%b/%Y %H:%M:%S", &time_info_);
  return buffer;
}

tm DateTime::GetTimeStruct() const
{
  return time_info_;
}

std::string DateTime::AbbreviatedMonthName() const
{
  char buffer[4];
  strftime(buffer, 4, "%b", &time_info_);
  return buffer;
}

std::string DateTime::AbbreviatedWeekdayName() const
{
  char buffer[4];
  strftime(buffer, 4, "%a", &time_info_);
  return buffer;
}

}  // namespace core::utils
