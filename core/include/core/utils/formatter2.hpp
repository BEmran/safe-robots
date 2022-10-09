// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_FORMATTER2_HPP_
#define CORE_UTILS_FORMATTER2_HPP_

#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

#include "core/utils/date_time.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/modifier.hpp"

namespace core::utils {

/**
 * @brief template function to convert tuple values to string using stream
 * @details https://en.cppreference.com/w/cpp/utility/integer_sequence
 * @param t tuple to be printed
 * @return std::string a string of tuple values as "[t0]...[tn]"
 */
template <class Tuple, std::size_t... Is>
std::string TupleToString(const Tuple& t, std::index_sequence<Is...>) {
  std::stringstream ss;
  ((ss << "[" << std::get<Is>(t) << "]"), ...);
  return ss.str();
}

/**
 * @brief  Formatter Interface class used to present message in different format
 * (style)
 *
 */
class FormaterInterface {
 public:
  virtual ~FormaterInterface() = default;
  /**
   * @brief Format passed message
   *
   * @param msg message
   * @return std::string formatted message
   */
  virtual std::string Format(std::string_view msg) const = 0;
};

/**
 * @brief Concrete class of FormaterInterface used as null formater
 *
 */
class NullFormater : public FormaterInterface {
 public:
  // return message as it is
  std::string Format(std::string_view msg) const override {
    return msg.data();
  }
};

/**
 * @brief Concrete class of FormaterInterface used to append different
 * number/type of object to use when formatting messages.
 * @details it uses Variadic Templates to accept any number of variables of
 * different types
 *
 * @tparam Ts object of any type
 */
template <class... Ts>
class Formater : public FormaterInterface {
 public:
  /**
   * @brief Construct a new Formater object of various type
   *
   * @param ts types value
   */
  Formater(Ts... ts) : tuple_(ts...), size_{std::index_sequence_for<Ts...>{}} {
  }

  // format tuple values + passed message
  std::string Format(std::string_view msg) const override {
    using namespace std::string_literals;
    return TupleToString(tuple_, size_) + ": "s + msg.data();
    // return TupleToString(tuple_) + ": "s + msg.data();
  }

 private:
  std::tuple<Ts...> tuple_;
  std::index_sequence_for<Ts...> size_;
};

/**
 * @brief Concrete class of FormaterInterface similar to Formater but with time
 * information added when formatting the message
 *
 * @tparam Ts object of any type
 */
template <class... Ts>
class TimeFormater : public FormaterInterface {
 public:
  /**
   * @brief Construct a new Time Formater object of various type
   *
   * @param ts types value
   */
  TimeFormater(Ts... ts) : formater_(ts...) {
  }

  // format time + tuple values + passed message
  std::string Format(std::string_view msg) const override {
    return "[" + core::utils::DateTime().TimeToString() + "]" +
           formater_.Format(msg);
  }

 private:
  Formater<Ts...> formater_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_FORMATTER2_HPP_
