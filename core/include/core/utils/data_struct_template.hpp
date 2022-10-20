// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_STRUCT_TEMPLATE_HPP_
#define CORE_UTILS_DATA_STRUCT_TEMPLATE_HPP_

#include <iostream>
#include <string>
#include <vector>

namespace core::utils {

/**
 * @brief Interface class defines API for all data struct
 *
 */
struct DataStructInterface {
  /**
   * @brief Default destructor the Data Struct Interface object
   *
   */
  virtual ~DataStructInterface() = default;

  /**
   * @brief clear/restart data
   *
   */
  virtual void Clear() = 0;

  /**
   * @brief return a string represent the data in general
   *
   * @return std::string data label
   */
  virtual std::string Label() const = 0;

  /**
   * @brief return a string describe the internal structure of the data if
   * available
   *
   * @return std::string data header
   */
  virtual std::string Header() const = 0;

  /**
   * @brief convert the values of the data to string
   *
   * @return std::string data values as string
   */
  virtual std::string ToString() const = 0;

  /**
   * @brief generic method to set the the data
   *
   * @param dsi_ptr a pointer to data structure interface
   */
  virtual void Set(DataStructInterface* dsi_ptr) = 0;
};

/**
 * @brief A main class implements the DataStructInterface and defines the
 * typical methods of all DataStrutcs. Different DataType should have their own
 * implementation of the methods
 *
 * @tparam T data type
 */
template <class T>
struct DataStruct : public DataStructInterface {
 public:
  /**
   * @brief Default construct for Data Struct object
   *
   * @param label data label
   */
  explicit DataStruct(const std::string& label = "")
    : data_{}, label_{label} {};

  /**
   * @brief Copy construct for a new Data Struct object
   *
   * @param ds other data struct of same type
   */
  explicit DataStruct(const DataStruct<T>& ds)
    : data_{ds.Get()}, label_{ds.Label()} {
  }

  /**
   * @brief Default destructor the Data Struct object
   *
   */
  ~DataStruct() = default;

  /**
   * @brief Return the data
   *
   * @return T data object
   */
  T Get() const {
    return data_;
  }

  /**
   * @brief Return a reference of data that can be used to modify the data
   *
   * @return T& a reference of Data object
   */
  T& Get() {
    return data_;
  }

  /**
   * @brief Set the Data object using another data object
   *
   * @param new_data new data object
   */
  void Set(const T& new_data) {
    data_ = new_data;
  }

  /**
   * @brief Safe method to set Data object using a pointer to
   * DataStructInterface
   *
   * @param dsi_ptr a pointer to data structure interface
   * @throw std::bad_cast if pointer is not to the same Data type
   */
  void Set(DataStructInterface* dsi_ptr) override {
    if (auto ds = dynamic_cast<DataStruct<T>*>(dsi_ptr); ds != nullptr) {
      Set(ds->Get());  // safe to call
    } else {
      throw std::bad_cast();
    }
  }

  /**
   * @brief Clear and reset Data object using default constructor
   *
   */
  void Clear() override {
    data_ = T{};
  }

  /**
   * @brief Return overall string represent the label of the Data object
   *
   * @return std::string label
   */
  std::string Label() const override {
    return label_;
  }

  /**
   * @brief Return a header string describes the internal structure of the Data
   * object
   *
   * @return std::string header
   */
  std::string Header() const override {
    return label_;
  }

  /**
   * @brief Convert data to string
   *
   * @return std::string data as a string
   */
  std::string ToString() const override;
  // {
  //   return "";
  // }

  /**
   * @brief Overload stream operator to print DataStruct nicely
   *
   * @param os a reference stream object
   * @param ds DataStructure object
   * @return std::ostream& return output-stream reference
   */
  friend std::ostream& operator<<(std::ostream& os, const DataStruct<T>& ds) {
    return os << "[" << ds.Header() << "]: " << ds.ToString();
  }

 protected:
  /// @brief holds Data object initialized using default constructor
  T data_{};

  /// @brief holds data label
  std::string label_{};
};

/**
 * @brief A general template class used to add a string as an additional
 * parametrize for the DataStruct
 *
 * @tparam TYPE Data type
 * @tparam LABEL data label
 */
template <class TYPE, auto& LABEL>
struct LabelDataStruct : public DataStruct<TYPE> {
  /**
   * @brief Default construct of Label Data Struct object
   *
   */
  LabelDataStruct() : DataStruct<TYPE>(LABEL) {
  }
};

}  // namespace core::utils

/* print data details */
std::ostream& operator<<(std::ostream& os,
                         const core::utils::DataStructInterface* const dsi) {
  return os << "[" << dsi->Header() << "]: " << dsi->ToString();
}

#endif  // CORE_UTILS_DATA_STRUCT_TEMPLATE_HPP_
