#ifndef CORE_UTILS_CASH_HPP
#define CORE_UTILS_CASH_HPP

#include <memory>
#include <shared_mutex>

namespace core::utils {
/**
 * @brief simple class used to define a thread-safe way to handle data
 *
 */
template <class T>
class Cash {
 public:
  /**
   * @brief Construct a new Cash object
   *
   */
  Cash() = default;

  /**
   * @brief Destroy the Cash object
   *
   */
  ~Cash() = default;

  /**
   * @brief Set the data value
   * @details Only one thread/writer can set the data's value
   *
   * @param data new data
   */
  void Set(const T& data) {
    std::unique_lock lock(mutex_);
    *data_ptr_.get() = data;
  }

  /**
   * @brief Return the stored data
   * @details Multiple threads/readers can get the data's value at the same time
   *
   * @return T data
   */
  T Get() const {
    std::shared_lock lock(mutex_);
    return *data_ptr_.get();
  }

  /**
   * @brief Clear data by calling its default constructor
   *
   */
  void Clear() {
    std::unique_lock lock(mutex_);
    data_ptr_->Clear();
  }

 private:
  mutable std::shared_mutex mutex_;
  std::unique_ptr<T> data_ptr_ = std::make_unique<T>();
};

}  // namespace core::utils
#endif  // CORE_UTILS_CASH_HPP