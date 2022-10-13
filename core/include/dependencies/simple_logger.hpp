#ifndef DEPENDENCIES_SIMPLE_LOGGER_HPP_
#define DEPENDENCIES_SIMPLE_LOGGER_HPP_

#include <chrono>
#include <ctime>
#include <functional>
#include <iostream>
#include <string>
#include <string_view>
#include <type_traits>

namespace depend {
using GetTimeCB = std::function<std::string()>;

/**
 * @brief get current time
 *
 * @return tm* time in time_info_ struct
 */
tm* TimeNow() {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  return std::localtime(&now_time_t);
}

/**
 * @brief Format time_info_ struct to string by rapping strftime function
 *.
 * @param tm time structure contains a calendar date and time broken down into
 * its components.
 * @return std::string the formatted string
 */
std::string TimeInfo() {
  auto tm = TimeNow();
  constexpr auto kMaxSize = 12;
  std::string buffer(kMaxSize, ' ', std::allocator<char>());
  strftime(buffer.data(), buffer.size(), " [%H:%M:%S]", tm);
  return buffer;
}

/**
 * @brief Simple logger to be used when logging data
 * @details
 * the logged message will lok like: [header TimeInfo(): Msg]
 * For default header and TimeInfo, the logged message will lok like: [Logger
 * [xx:xx:xx]: Msg].  Before appending first logged data, the logger will append
 * a header and TimeInfo(). when std::endl is sent to logger, logger will
 * consider the message has ended and will append the header and time
 * information for the next data. If the previous logged message does not end
 * with std::endl, the Logger append the output stream with std::endl at
 * destruction.
 *
 */
class Logger {
  using endl_type = std::ostream&(std::ostream&);  // This is the key: std::endl
                                                   // is a template function,
                                                   // and this is the signature
                                                   // of that function (For
                                                   // std::ostream).
 public:
  static const std::string_view default_header;

  /**
   * @brief Default constructor to a new Logger object
   *
   */
  Logger() : Logger(default_header, std::cout, TimeInfo) {
  }

  /**
   * @brief Construct a new Logger object with a custom log header and output
   * stream
   *
   * @param header log header
   * @param os output stream
   */
  Logger(std::string_view header, std::ostream& os, GetTimeCB time_cb)
    : m_header(header), m_os(os), m_time_cb(time_cb) {
  }

  ~Logger() {
    // stream endl at the end of logging
    if (!m_first_msg) {
      m_os << std::endl;
    }
  }

  /**
   * @brief Stream data for std::endl only:
   *
   * @param endl end of line
   * @return Logger& logger reference
   */
  Logger& operator<<(endl_type endl) {
    m_first_msg = true;
    m_os << endl;
    return *this;
  }

  /**
   * @brief Stream data
   *
   * @tparam T any type
   * @param data data to be logged
   * @return Logger& logger reference
   */
  template <typename T>
  Logger& operator<<(const T& data) {
    if (m_first_msg) {
      m_os << m_header << m_time_cb() << ": ";
      m_first_msg = false;
    }
    m_os << data;
    return *this;
  }

 private:
  bool m_first_msg{true};
  const std::string m_header;
  std::ostream& m_os;
  GetTimeCB m_time_cb;
};

const std::string_view Logger::default_header = "Logger";

Logger Debug() {
  return Logger("DEBUG", std::cout, TimeInfo);
}

Logger Error() {
  return Logger("ERROR", std::cerr, TimeInfo);
}

Logger Log(const std::string_view& header, std::ostream& os,
           GetTimeCB time_cb) {
  return Logger(header, os, time_cb);
}

}  // namespace depend

#endif  // DEPENDENCIES_SIMPLE_LOGGER_HPP_