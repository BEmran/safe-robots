// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger.hpp"

#include "core/utils/exception.hpp"

namespace core::utils {

constexpr std::string_view kFilenameExten = "_logger.txt";

Logger::Logger(const std::vector<WriterFormatterPair>& writer_formatter)
  : Logger(writer_formatter, std::make_shared<NullExceptionFactory>()) {
}

Logger::Logger(const std::vector<WriterFormatterPair>& writer_formatter,
               std::shared_ptr<ExceptionFactory> expectation_factory)
  : writer_formatter_vec_(writer_formatter)
  , expectation_factory_(std::move(expectation_factory)) {
}

/* TODO: Instead of the logger Associate exception to LabeledModifier, so user
 * can select what kind of exception they want to throw when it is used/called
 */
void Logger::Log(const LabeledModifier& lm, const std::string& msg) {
  for (const auto& wf : writer_formatter_vec_) {
    const std::string str = lm.ToString() + ' ' + msg.data();
    Dump(wf, str);
  }
  ThrowExceptionForErrorEvent(lm.GetEventLevel(), msg.data());
}

void Logger::Log(const EventLevel event, std::string_view msg) {
  for (const auto& wf : writer_formatter_vec_) {
    const std::string str = EventLevelToString(event) + ' ' + msg.data();
    Dump(wf, str);
  }
  ThrowExceptionForErrorEvent(event, msg.data());
}

void Logger::Dump(const WriterFormatterPair& wf, std::string_view msg) {
  if (wf.writer && wf.formatter) {
    const auto formatted = wf.formatter->Format(msg);
    wf.writer->Write(formatted);
  }
}

void Logger::ThrowExceptionForErrorEvent(const EventLevel event,
                                         const std::string& msg) {
  if (event == EventLevel::ERROR || event == EventLevel::FATAL) {
    expectation_factory_->Throw(msg);
  }
}

std::string GetFilename(std::string_view name, std::string_view filename) {
  if (filename.empty()) {
    return std::string(name) + kFilenameExten.data();
  }
  return filename.data();
}

std::shared_ptr<Logger> CreateFileAndConsoleLogger(std::string_view name,
                                                   std::string_view filename) {
  auto filename_updated = GetFilename(name, filename);
  auto file_writer = std::make_shared<FileWriter>(filename_updated);
  auto console_writer = std::make_shared<Writer>(std::cout);
  const std::vector<WriterFormatterPair> writer_formatter_vec{
    {file_writer, std::make_shared<NullFormater>()},
    {
      console_writer,
      std::make_shared<TimeFormater<std::string_view>>(name),
    }};
  const auto except_fact = std::make_shared<ExceptionFactory>(name.data());
  return std::make_shared<Logger>(writer_formatter_vec, except_fact);
}

}  // namespace core::utils
