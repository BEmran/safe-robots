// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_LOGGER_HPP_
#define CORE_UTILS_DATA_LOGGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "core/utils/data.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/subject.hpp"

namespace core::utils {

class DataLogger {
 public:
  /**
   * @brief Construct a new Data Logger object using a logger
   *
   * @param logger shared ptr to a logger object
   */
  explicit DataLogger(std::shared_ptr<Logger> logger) : logger_(logger) {
  }

  virtual ~DataLogger() = default;

  /**
   * @brief Log data
   *
   */
  void Log() const {
    std::string str;
    for (const auto d : data_vec_) {
      str += d->ToString();
    }
    logger_->Log(lm_, str);
  }

  template <typename T>
  void Observe(std::shared_ptr<Subject<T>> subject) {
    const auto size = data_vec_.size();
    data_vec_.push_back(new T(subject->Get()));
    auto lambda = [this, size](const T& d) {
      std::cout << "lmabda call back for " << d.ToString() << std::endl;
      data_vec_[size] = new T(d);
    };
    auto cb = std::make_shared<ObserverCallback<T>>(lambda);
    subject->Register(cb);
  }

 protected:
  // /**
  //  * @brief Logs a message with specific LabeledModifier
  //  * @details this function is also called internally by all log_* functions
  //  *
  //  * @param lm labeled modifier which defines event and its label
  //  * @param msg message to be logged
  //  */
  // virtual void LogImpl(const LabeledModifier& lm, const std::string& msg)
  // const;

 private:
  std::shared_ptr<Logger> logger_;
  std::vector<Data*> data_vec_;
  LabeledModifier lm_ = InfoLabeledModifier();
};

// /**
//  * @brief Create a new NodeLogger with typical settings for console and file
//  * formatter
//  *
//  * @param name name of Exception factory header, also used to create logger
//  * filename as "<name>_logger.txt"
//  * @return std::shared_ptr<NodeLogger> NodeLogger shared_ptr object
//  */
// std::shared_ptr<NodeLogger> CreateNodeLogger(const std::string& name);

// /**
//  * @brief Create a System NodeLogger with a common logger
//  *
//  * @param header NodeLogger header, typically set to node name
//  * @return std::shared_ptr<NodeLogger> NodeLogger shared_ptr object
//  */
// std::shared_ptr<NodeLogger> CreateSystemNodeLogger(const std::string&
// header);

}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
