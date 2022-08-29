// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/writer_file.hpp"

#include <iostream>

#include "core/utils/exception.hpp"

namespace core::utils {
FileWriter::FileWriter(const std::string& filename)
  : filename_(filename), file_(std::make_shared<std::ofstream>()) {
  file_->open(filename, std::ios_base::out);
}

FileWriter::~FileWriter() {
  dump_mutex_.lock();
  if (!file_->is_open()) {
    file_->close();
  }
  dump_mutex_.unlock();
}

void FileWriter::Dump(const std::string& str) {
  DumpToFile(str);
}

void FileWriter::DumpToFile(const std::string& str) {
  dump_mutex_.lock();
  if (!file_->is_open()) {
    throw Exception("FileWriter Can't open a file: " + filename_);
  }
  *file_ << str << std::endl;
  dump_mutex_.unlock();
}

}  // namespace core::utils
