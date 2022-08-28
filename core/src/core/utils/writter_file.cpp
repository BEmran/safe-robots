#include "core/utils/writter_file.hpp"

#include <iostream>

#include "core/utils/exception.hpp"

namespace core::utils {
FileWritter::FileWritter(const std::string& filename)
  : filename_(filename), file_(std::make_shared<std::ofstream>()) {
  file_->open(filename, std::ios_base::out);
}

FileWritter::~FileWritter() {
  dump_mutex_.lock();
  if (!file_->is_open()) {
    file_->close();
  }
  dump_mutex_.unlock();
}

void FileWritter::dump(const std::string& str) {
  DumpToFile(str);
}

void FileWritter::DumpToFile(const std::string& str) {
  dump_mutex_.lock();
  if (!file_->is_open()) {
    throw Exception("FileWritter Can't open a file: " + filename_);
  }
  *file_ << str << std::endl;
  dump_mutex_.unlock();
}

}  // namespace core::utils