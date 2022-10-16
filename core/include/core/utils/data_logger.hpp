// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_LOGGER_HPP_
#define CORE_UTILS_DATA_LOGGER_HPP_

#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "core/utils/data.hpp"
#include "core/utils/logger.hpp"
#include "core/utils/subject.hpp"

namespace core::utils {

/**
 * @brief Class to log a set of Data registered by observing their corresponding
 * Subject Object
 *
 */
class DataLogger {
 public:
  /**
   * @brief Construct a new Data Logger object using a logger
   *
   * @param logger shared ptr to a logger object
   */
  explicit DataLogger(std::shared_ptr<Logger> logger)
    : DataLogger(logger, InfoLabeledModifier()) {
  }

  /**
   * @brief Construct a new Data Logger object using a logger
   *
   * @param logger shared ptr to a logger object
   * @param lm labeled data modifier to use when logging the data
   */
  DataLogger(std::shared_ptr<Logger> logger, LabeledModifier lm)
    : logger_(logger), lm_(lm) {
  }

  virtual ~DataLogger() {
  }

  /**
   * @brief Log all data using internal Logger and LabeledModifier
   *
   */
  void Log() const {
    const std::string msg_data = ToString(data_vec_);
    logger_->Log(lm_, msg_data);
  }

  /**
   * @brief Observe passed subject
   * @details It creates a new data entree. then create an ObserverCB, which is
   * a lambda hocked up to the newly created data entree. Finally register the
   * ObserverCB at the subject the subject
   *
   * @tparam T subject's type (type of the data own by the subject)
   * @param subject subject to be observed
   */
  template <typename T>
  void Observe(std::shared_ptr<Subject<T>> subject) {
    const size_t new_data_idx = CreateAndInitNewData<T>(subject);
    auto observer_cb = CreateObserverCBAt<T>(new_data_idx);
    subject->Register(observer_cb);
  }

 protected:
  /**
   * @brief Create a And Initialize New Data object using the passed Subject
   *
   * @tparam T data type (subject type)
   * @param subject subject to get initial data from
   * @return size_t index of the newly created data
   */
  template <typename T>
  size_t CreateAndInitNewData(std::shared_ptr<Subject<T>> subject) {
    data_vec_.push_back(std::make_shared<T>(subject->Get()));
    const size_t data_idx = data_vec_.size() - 1;
    return data_idx;
  }

  /**
   * @brief Create a new observer callback to register it at a desired subject
   *
   * @tparam T data type (subject type)
   * @param idx index of the newly created data
   * @return  std::shared_ptr<ObserverCB<T>> observer callback
   */
  template <typename T>
  std::shared_ptr<ObserverCB<T>> CreateObserverCBAt(const size_t idx) {
    auto lambda = [this, idx](const T& data) {
      data_vec_[idx] = std::make_shared<T>(data);
    };
    return std::make_shared<ObserverCB<T>>(lambda);
  }

 private:
  /// @brief logger to be used when logging the data
  std::shared_ptr<Logger> logger_;

  /// @brief LabeledModifier to use when logging the data
  LabeledModifier lm_ = InfoLabeledModifier();

  /// @brief data vector to hold latest data values
  std::vector<std::shared_ptr<Data>> data_vec_;
};
}  // namespace core::utils

#endif  // CORE_UTILS_NODE_LOGGER_HPP_
