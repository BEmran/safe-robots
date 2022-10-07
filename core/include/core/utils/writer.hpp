// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_WRITER_HPP_
#define CORE_UTILS_WRITER_HPP_

#include <iostream>
#include <string_view>

namespace core::utils {

/**
 * @brief A Writer class is a lower level class that writes on output stream
 * (std::cout, std::cerr, stringstream)
 *
 */
class Writer {
 public:
  using endl_type = std::ostream&(std::ostream&);

  /**
   * @brief Construct a default Console Writer object with std::cout as
   * ostream
   *
   */
  Writer();

  /**
   * @brief Construct a generic Console Writer object with the passed ostream
   *
   * @param os ostream to be used
   */
  Writer(std::ostream& os);

  /**
   * @brief Destroy the Writer object
   *
   */
  virtual ~Writer() = default;

  /**
   * @brief Write string data to stream appended by new line
   *
   * @param str string to write
   */
  void Write(std::string_view str) const;

  /**
   * @brief Stream data to all types
   *
   * @tparam T any type
   * @param data data to be written
   * @return Writer& Writer reference
   */
  template <typename T>
  Writer& operator<<(const T& data) {
    if (IsReady()) {
      os_ << data;
    }
    return *this;
  }

  /**
   * @brief Stream data for std::endl only
   *
   * @param endl end of line
   * @return Logger& logger reference
   */
  Writer& operator<<(endl_type endl);

 protected:
  /**
   * @brief Checks if stream is ready to write data
   *
   * @return true if stream ready
   * @return false otherwise
   */
  virtual bool IsReady() const;

 private:
  /**
   * @brief reference to output stream
   *
   */
  std::ostream& os_;
};
}  // namespace core::utils

#endif  // CORE_UTILS_WRITER_HPP_
