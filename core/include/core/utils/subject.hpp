// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SUBJECT_HPP_
#define CORE_UTILS_SUBJECT_HPP_

#include <functional>
#include <memory>
#include <optional>
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
using ObserverCB = std::function<void(const T& data)>;

/**
 * @brief General registry class holds observers' callback to inform when
 * internal data changes
 *
 * @tparam T internal data type
 */
template <typename T>
class Subject {
 private:
  using Registry = std::set<std::shared_ptr<ObserverCB<T>>>;

 public:
  /**
   * @brief Construct a new Subject object
   *
   * @param name subject name
   */
  explicit Subject(const std::string& name)
    : node_(std::make_unique<Node>(name)) {
  }

  /**
   * @brief Register new observer callback if not registered before
   *
   * @param cb observer callback
   */
  void Register(std::shared_ptr<ObserverCB<T>> cb) {
    if (FindMatch(cb)) {
      node_->GetNodeLogger()->Warn(
        "Failed to register an observer callback; it has been registered "
        "before");
    } else {
      register_.insert(cb);
      node_->GetNodeLogger()->Debug(
        "Successfully register a new observer callback");
    }
  }

  /**
   * @brief Unregister observer callback if registered before
   *
   * @param cb observer callback
   */
  void Unregister(std::shared_ptr<ObserverCB<T>> cb) {
    auto it = FindMatch(cb);
    if (it) {
      register_.erase(it.value());
      node_->GetNodeLogger()->Debug(
        "Successfully unregister an observer callback");
    } else {
      node_->GetNodeLogger()->Warn(
        "Failed to find a match for an observer callback to unregister");
    }
  }

  /**
   * @brief Find a match of the passed Observer callback in the registry
   *
   * @param cb observer callback
   * @return std::optional<Registry::iterator> iteration of the founded match
   */
  std::optional<typename Registry::iterator>
  FindMatch(std::shared_ptr<ObserverCB<T>> cb) {
    auto it = register_.find(cb);
    if (it == register_.end()) {
      return {};
    }
    return it;
  }

  /**
   * @brief Set internal data and Notify observers
   *
   * @param data new data
   */
  void SetAndNotify(const T& data) {
    cash_data_.Set(data);
    Notify();
  }

  /**
   * @brief Notify all observers and give them a copy of internal data
   *
   */
  void Notify() const {
    const auto data = Get();
    std::for_each(register_.begin(), register_.end(),
                  [&data](auto obs) { (*obs)(data); });
  }

  /**
   * @brief Get internal data
   *
   * @return T stored data
   */
  T Get() const {
    return cash_data_.Get();
  }

 private:
  /// @brief node used to log data
  std::unique_ptr<Node> node_;

  /// @brief cash object to safely store and retrieve object data
  Cash<T> cash_data_;

  /// @brief set of shared_ptr to all ObserverCallback function
  Registry register_;
};

}  // namespace core::utils
#endif  // CORE_UTILS_SUBJECT_HPP_
