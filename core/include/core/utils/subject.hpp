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

template <typename T>

using ObserverCallback = std::function<void(const T& data)>;

template <typename T>
class Subject {
 public:
  explicit Subject(const std::string& name)
    : node_(std::make_unique<Node>(CreateSystemNode(name))) {
  }

  void Register(std::shared_ptr<ObserverCallback<T>> cb) {
    auto it = observers_.find(cb);
    // TODO(Bara): not needed for Set container
    if (it != observers_.end()) {
      node_->GetLogger()->LogWarn("This observer has been registered before.");
      return;
    }

    observers_.insert(cb);
    node_->GetLogger()->LogDebug("Registered new observer.");
  }

  void Unregister(std::shared_ptr<ObserverCallback<T>> cb) {
    auto it = observers_.find(cb);

    if (it == observers_.end()) {
      node_->GetLogger()->LogWarn("This observer was not registered.");
      return;
    }

    observers_.erase(it);
    node_->GetLogger()->LogDebug("Unregistered an observer.");
  }

  void Inform() const {
    const auto data = Get();
    std::for_each(observers_.begin(), observers_.end(),
                  [&data](auto obs) { (*obs)(data); });
  }

  void Set(const T& data) {
    cash_data_.Set(data);
    Inform();
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
