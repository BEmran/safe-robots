// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/logger.hpp"

#include "core/utils/exception.hpp"

namespace core::utils {

constexpr std::string_view kFilenameExten = "_logger.txt";

std::string PrintedName(std::string_view name) {
  using namespace std::literals;
  if (name.empty()) {
    return {};
  } else {
    return "["s + name.data() + "] "s;
  }
}

Logger::Logger(const LoggerConfig& config)
  : printed_name_{PrintedName(config.name)}
  , logging_level_{config.level}
  , writer_formatter_vec_{config.wf_pairs}
  , expectation_factory_{config.expectation_factory} {
}

void Logger::Log(const LabeledModifier& lm, std::string_view msg) const {
  if (lm.GetEventLevel() >= logging_level_) {
    LogImp(lm, msg);
  }
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
                            const LabeledModifier& lm,
                            std::string_view msg) const {
  const std::string labeled_msg = printed_name_ + msg.data();
  const auto formatted = wf.formatter.Format(lm, labeled_msg);
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
 * @return WriterFormatterPair writer and formatter
 */
WriterFormatterPair CreateWriterFormatterPair(std::ostream& os) {
  auto writer = std::make_shared<Writer>(os);
  auto formatter = CreateTimeLabelModifierFormatter();
  return {writer, formatter};
}

/**
 * @brief Create a FileWriter and pair it with TimeFormatter
 *
 * @param name logger name
 * @param filename possible filename
 * @return WriterFormatterPair writer and formatter
 */
WriterFormatterPair CreateWriterFormatterPair(std::string_view filename) {
  auto writer = std::make_shared<FileWriter>(filename);
  auto formatter = CreateTimeLabelFormatter();
  return {writer, formatter};
}
}  // namespace

Logger CreateStreamLogger(std::string_view name, std::ostream& os) {
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  LoggerConfig config;
  config.name = name;
  config.level = EventLevel::DEBUG;
  config.wf_pairs = {CreateWriterFormatterPair(os)};
  config.expectation_factory = std::make_shared<ExceptionFactory>(name);
  return Logger(config);
}

Logger CreateFileLogger(std::string_view name, std::string_view filename) {
  const auto except_fact = std::make_shared<ExceptionFactory>(name);
  const auto filename_updated = GenerateFilename(name, filename);
  LoggerConfig config;
  config.name = name;
  config.level = EventLevel::DEBUG;
  config.wf_pairs = {CreateWriterFormatterPair(filename_updated)};
  config.expectation_factory = std::make_shared<ExceptionFactory>(name);
  return Logger(config);
}

Logger CreateStreamAndFileLogger(std::string_view name, std::ostream& os,
                                 std::string_view filename) {
  const auto filename_updated = GenerateFilename(name, filename);
  LoggerConfig config;
  config.name = name;
  config.level = EventLevel::DEBUG;
  config.wf_pairs = {CreateWriterFormatterPair(os),
                     CreateWriterFormatterPair(filename_updated)};
  config.expectation_factory = std::make_shared<ExceptionFactory>(name);
  return Logger(config);
}

}  // namespace core::utils
