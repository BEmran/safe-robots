// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_MACROS_HPP_
#define CORE_UTILS_LOGGER_MACROS_HPP_

#include "core/utils/event_level.hpp"
#include "core/utils/logger_node.hpp"

#define SYS_LOG_DEBUG(msg) core::utils::SysLog()->Debug(msg)

#define SYS_LOG_INFO(msg) core::utils::SysLog()->Info(msg)

#define SYS_LOG_WARN(msg) core::utils::SysLog()->Warn(msg)

#define SYS_LOG_ERROR(msg)                                                     \
  core::utils::SysLog()->Error(LOG_INFORMATION_STRING + msg)

#define SYS_LOG_FATAL(msg)                                                     \
  core::utils::SysLog()->Fatal(LOG_INFORMATION_STRING + msg)

#endif  // CORE_UTILS_LOGGER_MACROS_HPP_
