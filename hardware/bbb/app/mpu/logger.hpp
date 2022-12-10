#ifndef RC_LOGGER_HPP
#define RC_LOGGER_HPP

#include <iostream>
#include <string>

std::ostream& SYS_LOG_ERROR(const std::string& msg);

std::ostream& SYS_LOG_WARN(const std::string& msg);

std::ostream& SYS_LOG_DEBUG(const std::string& msg);

#endif  // RC_LOGGER_HPP