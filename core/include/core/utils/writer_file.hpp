// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_WRITER_FILE_HPP_
#define CORE_UTILS_WRITER_FILE_HPP_

#include <fstream>
#include <string>
#include <string_view>

#include "core/utils/writer.hpp"

namespace core::utils {
/**
 * @brief An extention class to Writer used to write on file stream
 *
 */
class FileWriter : public Writer {
 public:
  FileWriter(std::string_view filename);

  ~FileWriter();

 protected:
  bool IsReady() const override;

 private:
  std::string filename_;
  std::ofstream file_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITER_FILE_HPP_