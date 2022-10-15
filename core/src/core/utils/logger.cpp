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

/* TODO: Instead of the logger Associate exception to LabeledModifier, so user
 * can select what kind of exception they want to throw when it is used/called
 */
void Logger::Log(const LabeledModifier& lm, std::string_view msg) const {
  LogImp(lm, msg);
  ThrowExceptionForCriticalEvent(lm.GetEventLevel(), msg);
}

void Logger::Log(const EventLevel event, std::string_view msg) const {
  Log(LabeledModifier(event), msg);
}

void Logger::LogImp(const LabeledModifier& lm, std::string_view msg) const {
  for (size_t idx = 0; idx < writer_formatter_vec_.size(); ++idx) {
    auto& wf = writer_formatter_vec_[idx];
    if (wf.writer) {
      FormatAndWrite(wf, lm, msg);
    } else {
      std::cerr << "Logger: WriterFormatterPair #[" << idx << "] is invalid"
                << std::endl;
    }
  }
}

void Logger::FormatAndWrite(const WriterFormatterPair& wf,
                            const LabeledModifier& lm, std::string_view msg) {
  const auto formatted = wf.formatter.Format(lm, msg);
  wf.writer->Write(formatted);
}

void Logger::ThrowExceptionForCriticalEvent(const EventLevel event,
                                            std::string_view msg) const {
  if (IsCritical(event)) {
    expectation_factory_->Throw(msg.data());
  }
}

namespace {
/**
 * @brief Generate filename using the passed name and filename which could be
 * empty
 *
 * @param name name of object
 * @param filename possible filename
 * @return std::string resultant filename
 */
std::string GenerateFilename(std::string_view name, std::string_view filename) {
  if (filename.empty()) {
    return std::string(name) + kFilenameExten.data();
  }
  return filename.data();
}

/**
 * @brief Create a stream Writer and pair it with TimeFormatter
 *
 * @param name logger name
 * @param os output stream
 * @return WriterFormatterPair writer and formater
 */
WriterFormatterPair CreateWriterFormatterPair(std::string_view name,
                                              std::ostream& os) {
  auto writer = std::make_shared<Writer>(os);
  auto formater = CreateTimeLabelModifierFormatter();
  return {writer, formater};
}

/**
 * @brief Create a FileWriter and pair it with TimeFormatter
 *
 * @param name logger name
 * @param filename possible filename
 * @return WriterFormatterPair writer and formater
 */
WriterFormatterPair CreateWriterFormatterPair(std::string_view name,
                                              std::string_view filename) {
  const auto filename_updated = GenerateFilename(name, filename);
  auto writer = std::make_shared<FileWriter>(filename_updated);
  auto formater = CreateTimeLabelFormatter();
  return {writer, formater};
}
}  // namespace

Logger CreateStreamLogger(std::string_view name, std::ostream& os) {
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  return Logger({CreateWriterFormatterPair(name, os)}, except_fact);
}

Logger CreateFileLogger(std::string_view name, std::string_view filename) {
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  return Logger({CreateWriterFormatterPair(name, filename)}, except_fact);
}

Logger CreateStreamAndFileLogger(std::string_view name, std::ostream& os,
                                 std::string_view filename) {
  std::vector<WriterFormatterPair> wf_vec{
    CreateWriterFormatterPair(name, os),
    CreateWriterFormatterPair(name, filename)};
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  return Logger(wf_vec, except_fact);
}

}  // namespace core::utils
