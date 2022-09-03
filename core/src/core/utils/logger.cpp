// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger.hpp"

#include "core/utils/exception.hpp"

namespace core::utils {
Logger::Logger(const std::string& filename)
  : Logger(filename, std::make_shared<NullExceptionFactory>()) {
}

Logger::Logger(const std::string& filename,
               std::shared_ptr<ExceptionFactory> expectation_factory)
  : Logger(filename, CreateNullFormatter(), CreateNullFormatter(),
           std::move(expectation_factory)) {
}

Logger::Logger(const std::string& filename,
               std::shared_ptr<Formatter> file_formater,
               std::shared_ptr<Formatter> console_formater)
  : Logger(filename, std::move(file_formater), std::move(console_formater),
           std::make_shared<NullExceptionFactory>()) {
}

Logger::Logger(const std::string& filename,
               std::shared_ptr<Formatter> file_formater,
               std::shared_ptr<Formatter> console_formater,
               std::shared_ptr<ExceptionFactory> expectation_factory)
  : file_writer_(new FileWriter(filename))
  , console_writer_(new ConsoleWriter())
  , file_formater_(std::move(file_formater))
  , console_formater_(std::move(console_formater))
  , expectation_factory_(std::move(expectation_factory)) {
}

void Logger::Log(const LabeledModifier& lm, const std::string& msg) {
  const auto file_str = file_formater_->Format(lm, msg);
  file_writer_->Dump(file_str);
  const auto console_str = console_formater_->Format(lm, msg);
  console_writer_->Dump(console_str);
  /* TODO: Instead of the logger Associate exception to LabeledModifier, so user
   * can select what kind of exception they want to throw when it is used/called
   */
  ThrowExceptionForErrorEvent(lm.GetEventLevel(), msg);
}

void Logger::ThrowExceptionForErrorEvent(const EventLevel event,
                                         const std::string& msg) {
  if (event == EventLevel::EL_ERROR) {
    expectation_factory_->Throw(msg);
  }
}

std::shared_ptr<Logger> CreateDefaultLogger(const std::string& name,
                                            const std::string& filename) {
  const auto file_fmt = CreateTimeLabelFormatter();
  const auto console_fmt = CreateTimeLabelModifierFormatter();
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  return std::make_shared<Logger>(filename, file_fmt, console_fmt, except_fact);
}

}  // namespace core::utils
