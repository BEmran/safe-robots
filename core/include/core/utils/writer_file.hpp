// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_WRITER_FILE_HPP_
#define CORE_UTILS_WRITER_FILE_HPP_

#include <fstream>
#include <memory>
#include <mutex>  // NOLINT [build/c++11] TODO(Bara)
#include <string>

#include "core/utils/writer.hpp"

namespace core::utils {
/**
 * @brief A concrete class of Writer used to log data to a file using ofstream
 * object.
 *
 */
class FileWriter : public Writer {
 public:
  /**
   * @brief Construct a new File Writer object with a specific file name
   *
   * @param filename name of the file to be created
   *
   * TODO: create a robust way to create a file with date tag or without
   */
  explicit FileWriter(const std::string& filename);

  /**
   * @brief Destroy the File Writer object
   *
   */
  ~FileWriter() override;

  /* Writer Interface */
  void Dump(const std::string& str) const override;

 private:
  std::string filename_;
  std::shared_ptr<std::ofstream> file_;
  mutable std::mutex dump_mutex_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITER_FILE_HPP_
