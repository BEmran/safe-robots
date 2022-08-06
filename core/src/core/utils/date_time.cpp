#include "core/utils/date_time.hpp"

#include <iostream>
#include <map>

#include "core/utils/exception.hpp"

namespace core
{
namespace utils
{
namespace
{
tm craete_tm_struct()
{
  const auto raw_time = time(nullptr);
  return *localtime(&raw_time);
}
}  // namespace

long time_in_seconds()
{
  const auto now = std::chrono::system_clock::now();
  const auto epoch = now.time_since_epoch();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch).count();
  return sec;
}

std::string time_in_seconds_string()
{
  char buff[15];
  sprintf(buff, "%ld", time_in_seconds());
  return buff;
}

std::string generate_file_name(const DateTime dt)
{
  char buffer[21];
  tm time_struct = dt.get_time_struct();
  strftime(buffer, 21, "%d-%b-%Y_%H-%M-%S", &time_struct);
  return buffer;
}

/*****************************************************************************/

DateTime::DateTime() : DateTime(craete_tm_struct())
{
}

DateTime::DateTime(const tm& time_struct) : time_info_(time_struct)
{
}

std::string DateTime::date_to_string()
{
  char buffer[11];
  strftime(buffer, 11, "%d/%m/%Y", &time_info_);
  return buffer;
}

std::string DateTime::time_to_string()
{
  char buffer[9];
  strftime(buffer, 9, "%H:%M:%S", &time_info_);
  return buffer;
}

std::string DateTime::pretty()
{
  char buffer[31];
  strftime(buffer, 31, "%a %d/%b/%Y %H:%M:%S", &time_info_);
  return buffer;
}

tm DateTime::get_time_struct() const
{
  return time_info_;
}

std::string DateTime::abbreviated_month_name() const
{
  char buffer[4];
  strftime(buffer, 4, "%b", &time_info_);
  return buffer;
}

std::string DateTime::abbreviated_weekday_name() const
{
  char buffer[4];
  strftime(buffer, 4, "%a", &time_info_);
  return buffer;
}

}  // namespace utils
}  // namespace core
