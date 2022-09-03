// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/writer_console.hpp"

#include <iostream>

namespace core::utils {
void ConsoleWriter::Dump(const std::string& str) const {
  std::cout << str << std::endl;
}

}  // namespace core::utils
