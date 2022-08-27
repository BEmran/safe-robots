#ifndef CORE_UTILS_DATE_TIME_HPP
#define CORE_UTILS_DATE_TIME_HPP

#include <chrono>
#include <ctime>
#include <string>

namespace core::utils
{
/**
 * @brief Gets local machine date and time
 * @details the time structure used here could be found here:
 * https://www.cplusplus.com/reference/ctime/tm/
 * tm_sec	  int	seconds after the minute	0-61*
 * tm_min	  int	minutes after the hour	0-59
 * tm_hour	int	hours since midnight	0-23
 * tm_mday	int	day of the month	1-31
 * tm_mon	  int	months since January	0-11
 * tm_year	int	years since 1900
 * tm_wday	int	days since Sunday	0-6
 * tm_yday	int	days since January 1	0-365
 * tm_isdst	int	Daylight Saving Time flag
 */
class DateTime
{
 public:
  /**
   * @brief Construct a new DateTime object using the current machine time
   *
   */
  DateTime();

  /**
   * @brief Construct a new DateTime object using timestructure (tm) by copying
   * its values
   *
   * @param time_struct struct tm defined in ctime header file
   */
  explicit DateTime(const tm& time_struct);

  /**
   * @brief returns a string with date information
   * @details the string is in format of [year](spe)[month](sep)[day]
   * @return std::string generated date
   */
  std::string DateToString();

  /**
   * @brief returns a string with time information
   * @details the string is in format of [hour](spe)[minutes](sep)[seconds]
   * @return std::string generated time
   */
  std::string TimeToString();

  /**
   * @brief returns a string with date and time information in a pretty way
   * @details the returned string in the format of [day name] [day]/[month name]
   * @return std::string date and time
   */
  std::string Pretty();

  /**
   * @brief Get time structure contains a date and time broken down into its
   * components.
   *
   * @return tm time structure
   */
  tm GetTimeStruct() const;

  /**
   * @brief returns the abbreviated month name
   *
   * @return std::string 3 chars represent the abbreviated month name
   */
  std::string AbbreviatedMonthName() const;

  /**
   * @brief returns the abbreviated weekday name
   *
   * @return std::string 3 chars represent the weekday month name
   */
  std::string AbbreviatedWeekdayName() const;

 private:
  tm time_info_;  // Structure containing a calendar date and time broken down
                  // into its components.
};

/**
 * @brief returns the current time since the start of its clock in milliseconds as
 * long long integer
 *
 * @return int64_t time in milliseconds
 */
int64_t TimeInMilliSec();

/**
 * @brief returns the current time since the start of its clock in seconds as
 * long long integer
 *
 * @return int64_t time in seconds
 */
int64_t TimeInSeconds();

/**
 * @brief returns the current time since the start of its clock in milliseconds as
 * long long integer
 *
 * @return int64_t time in milliseconds
 */
int64_t TimeInMilliSec();

/**
 * @brief returns the current time since the start of its clock in seconds as
 * string format
 *
 * @return std::string time in seconds
 */
std::string TimeInSecondsString();

/**
 * @brief A standard way to generate a filename using DateTime information
 *
 * @param dt use the passed DateTime object or using current DateTime
 * @return std::string a standard filename
 */
std::string GenerateFileName(const DateTime dt = DateTime());

}  // namespace core::utils
#endif  // CORE_UTILS_DATE_TIME_HPP