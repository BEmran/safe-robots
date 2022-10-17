// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_MACROS_HPP_
#define CORE_UTILS_LOGGER_MACROS_HPP_

#include "core/utils/event_level.hpp"
#include "core/utils/logger.hpp"

#define SYS_LOG_DEBUG(msg)                                                     \
  core::utils::SystemLogger().Log(core::utils::EventLevel::DEBUG, msg);

#define SYS_LOG_ERROR(msg)                                                     \
  core::utils::SystemLogger().Log(core::utils::EventLevel::ERROR, msg);

#define SYS_LOG_FATAL(msg)                                                     \
  core::utils::SystemLogger().Log(core::utils::EventLevel::FATAL, msg);

#define SYS_LOG_INFO(msg)                                                      \
  core::utils::SystemLogger().Log(core::utils::EventLevel::INFO, msg);

#define SYS_LOG_WARN(msg)                                                      \
  core::utils::SystemLogger().Log(core::utils::EventLevel::WARN, msg);

#endif  // CORE_UTILS_LOGGER_MACROS_HPP_
