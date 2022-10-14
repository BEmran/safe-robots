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
  , expectation_factory_(expectation_factory) {
}

// void Logger::Debug(std::string_view msg) const {
//   Log(labeled_modifiers_.debug, msg.data());
// }

// void Logger::Error(std::string_view msg) const {
//   Log(labeled_modifiers_.error, msg.data());
// }

// void Logger::Fatal(std::string_view msg) const {
//   Log(labeled_modifiers_.fatal, msg.data());
// }

// void Logger::Info(std::string_view msg) const {
//   Log(labeled_modifiers_.info, msg.data());
// }

// void Logger::Warn(std::string_view msg) const {
//   Log(labeled_modifiers_.warn, msg.data());
// }

/* TODO: Instead of the logger Associate exception to LabeledModifier, so user
 * can select what kind of exception they want to throw when it is used/called
 */
void Logger::Log(const LabeledModifier& lm, std::string_view msg) const {
  const std::string event_str = "[" + lm.ToString() + "] ";
  LogImp(lm.GetEventLevel(), event_str, msg);
}

void Logger::Log(const EventLevel event, std::string_view msg) const {
  const std::string event_str = "[" + EventLevelToString(event) + "] ";
  LogImp(event, event_str, msg);
}

void Logger::LogImp(const EventLevel event, const std::string& event_str,
                    std::string_view msg) const {
  const std::string str = event_str + msg.data();
  for (const auto& wf : writer_formatter_vec_) {
    Dump(wf, str);
  }
  ThrowExceptionForErrorEvent(event, msg);
}

void Logger::Dump(const WriterFormatterPair& wf, const std::string& msg) const {
  if (wf.writer && wf.formatter) {
    const auto formatted = wf.formatter->Format(msg);
    wf.writer->Write(formatted);
  }
}

void Logger::ThrowExceptionForErrorEvent(const EventLevel event,
                                         std::string_view msg) const {
  if (event == EventLevel::ERROR || event == EventLevel::FATAL) {
    expectation_factory_->Throw(msg.data());
  }
}

std::string GetFilename(std::string_view name, std::string_view filename) {
  if (filename.empty()) {
    return std::string(name) + kFilenameExten.data();
  }
  return filename.data();
}

Logger CreateFileAndConsoleLogger(std::string_view name,
                                  std::string_view filename) {
  auto filename_updated = GetFilename(name, filename);
  auto file_writer = std::make_shared<FileWriter>(filename_updated);
  auto console_writer = std::make_shared<Writer>(std::cout);
  auto formater = std::make_shared<TimeFormater<std::string_view>>(name);
  std::vector<WriterFormatterPair> writer_formatter_vec{
    {file_writer, formater}, {console_writer, formater}};
  const auto except_fact = std::make_shared<ExceptionFactory>(name.data());
  return Logger(writer_formatter_vec, except_fact);
}

}  // namespace core::utils
