// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/writer_file.hpp"

#include <iostream>

#include "core/utils/exception.hpp"

namespace core::utils {
FileWriter::FileWriter(std::string_view filename)
  : Writer(file_), filename_{filename} {
  file_.open(filename_, std::ios_base::out);
  if (!file_.is_open()) {
    throw Exception("FileWriter can't open file: " + filename_);
  }
}

FileWriter::~FileWriter() {
  if (!file_.is_open()) {
    file_.close();
  }
}

bool FileWriter::IsReady() const {
  if (!file_.is_open()) {
    std::cerr << "FileWriter cannot open file: " << filename_ << std::endl;
    return false;
  }
  return true;
}

}  // namespace core::utils
