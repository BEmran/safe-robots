// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/writer.hpp"

namespace core::utils {

Writer::Writer() : Writer(std::cout) {
}

Writer::Writer(std::ostream& os) : os_(os) {
}

void Writer::Write(std::string_view str) const {
  if (IsReady()) {
    os_ << str << std::flush;
  }
}

Writer& Writer::operator<<(endl_type endl) {
  if (IsReady()) {
    os_ << endl;
  }
  return *this;
}

bool Writer::IsReady() const {
  return true;
}

}  // namespace core::utils
