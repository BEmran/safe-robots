// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SUBJECT_HPP_
#define CORE_UTILS_SUBJECT_HPP_

#include <functional>
#include <memory>
#include <set>
#include <string>

#include "core/utils/cash.hpp"
#include "core/utils/node.hpp"

namespace core::utils {

/**
 * @brief observer call back to register in a Subject
 *
 * @tparam T subject data type
 */
template <typename T>
using ObserverCallback = std::function<void(const T& data)>;

/**
 * @brief General Subject class holds observers' callback to inform when
 * internal data changes
 *
 * @tparam T internal data type
 */
template <typename T>
class Subject {
 public:
  /**
   * @brief Construct a new Subject object
   *
   * @param name subject name
   */
  explicit Subject(const std::string& name)
    : node_(std::make_unique<Node>(CreatNodeLoggerUsingSystemLogger(name))) {
  }

  void Register(std::shared_ptr<ObserverCallback<T>> cb) {
    auto it = observers_.find(cb);
    // TODO(Bara): not needed for Set container
    if (it != observers_.end()) {
      node_->GetLogger().Warn("This observer has been registered before");
    } else {
      observers_.insert(cb);
      node_->GetLogger().Debug("Registered new observer");
    }
  }

  void Unregister(std::shared_ptr<ObserverCallback<T>> cb) {
    auto it = observers_.find(cb);

    if (it == observers_.end()) {
      node_->GetLogger().Warn("This observer was not registered");
    } else {
      observers_.erase(it);
      node_->GetLogger().Debug("Unregistered an observer");
    }
  }

  void Notify() const {
    const auto data = Get();
    std::for_each(observers_.begin(), observers_.end(),
                  [&data](auto obs) { (*obs)(data); });
  }

  void Set(const T& data) {
    cash_data_.Set(data);
    Notify();
  }

  T Get() const {
    return cash_data_.Get();
  }

 private:
  std::unique_ptr<Node> node_;
  Cash<T> cash_data_;
  // store as unique_ptr to get its address for comparison specially lambda func
  std::set<std::shared_ptr<ObserverCallback<T>>> observers_;
};

}  // namespace core::utils
#endif  // CORE_UTILS_SUBJECT_HPP_
