// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger.hpp"

#include "core/utils/exception.hpp"

namespace core::utils {

Logger::Logger(const std::vector<WriterFormatterPair>& writer_formatter)
  : Logger(writer_formatter, std::make_shared<NullExceptionFactory>()) {
}

Logger::Logger(const std::vector<WriterFormatterPair>& writer_formatter,
               std::shared_ptr<ExceptionFactory> expectation_factory)
  : writer_formatter_vec_(writer_formatter)
  , expectation_factory_(std::move(expectation_factory)) {
}

void Logger::Log(const LabeledModifier& lm, const std::string& msg) {
  for (const auto& wf : writer_formatter_vec_) {
    Dump(wf, lm, msg);
  }

  /* TODO: Instead of the logger Associate exception to LabeledModifier, so user
   * can select what kind of exception they want to throw when it is used/called
   */
  ThrowExceptionForErrorEvent(lm.GetEventLevel(), msg);
}

void Logger::Dump(const WriterFormatterPair& wf, const LabeledModifier& lm,
                  const std::string& msg) {
  if (wf.writer) {
    const auto str = wf.formatter.Format(lm, msg);
    wf.writer->Dump(str);
  }
}

void Logger::ThrowExceptionForErrorEvent(const EventLevel event,
                                         const std::string& msg) {
  if (event == EventLevel::EL_ERROR) {
    expectation_factory_->Throw(msg);
  }
}

std::shared_ptr<Logger> CreateFileAndConsoleLogger(const std::string& name) {
  const auto filename = name + "_logger.txt";
  const auto writer_formatter_vec = {
    WriterFormatterPair{std::make_shared<FileWriter>(filename),
                        CreateTimeLabelFormatter()},
    WriterFormatterPair{std::make_shared<ConsoleWriter>(),
                        CreateTimeLabelModifierFormatter()},
  };
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  return std::make_shared<Logger>(writer_formatter_vec, except_fact);
}

}  // namespace core::utils
