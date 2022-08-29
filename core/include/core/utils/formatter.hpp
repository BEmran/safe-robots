// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_FORMATTER_HPP_
#define CORE_UTILS_FORMATTER_HPP_

#include <string>

#include "core/utils/date_time.hpp"
#include "core/utils/event_level.hpp"
#include "core/utils/terminal.hpp"

namespace core::utils {
/**
 * @brief Interface class used to present message in certain format (style)
 *
 */
class FormatterInterface {
 public:
  virtual ~FormatterInterface() = default;

  virtual std::string Format(const LabeledModifier& lm,
                             const std::string& msg) const = 0;
};

class NullFormatter : public FormatterInterface {
 public:
  ~NullFormatter() override = default;

  std::string Format(const LabeledModifier& lm,
                     const std::string& msg) const override;
};

class DefaultFormater : public FormatterInterface {
 public:
  explicit DefaultFormater(bool use_modifier = false);
  ~DefaultFormater() override = default;

  std::string Format(const LabeledModifier& lm,
                     const std::string& msg) const override;

  std::string AddLabeledModifier(const LabeledModifier& lm) const;

 protected:
  bool use_modifier_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_FORMATTER_HPP_
