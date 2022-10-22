// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger_stream.hpp"

namespace core::utils {

StreamLogger::StreamLogger(std::shared_ptr<Logger> logger,
                           const LabeledModifier& lm, std::string_view init_msg)
  : logger_(logger), lm_(lm) {
  oss_ << init_msg;
}

StreamLogger::~StreamLogger() noexcept(false) {
  logger_->Log(lm_, oss_.str());  // cppcheck-suppress exceptThrowInDestructor
}

StreamLogger& StreamLogger::operator<<(EndlType endl) {
  oss_ << endl;
  return *this;
}

}  // namespace core::utils
