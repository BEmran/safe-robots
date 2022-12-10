#include "logger.hpp"

std::ostream& SYS_LOG_ERROR(const std::string& msg) {
  return std::cout << "ERROR: " << msg;
}

std::ostream& SYS_LOG_WARN(const std::string& msg) {
  return std::cout << "WARNING: " << msg;
}

std::ostream& SYS_LOG_DEBUG(const std::string& msg) {
  return std::cout << "DEBUG: " << msg;
}