// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/event_level.hpp"

#include <map>

#include "core/utils/exception.hpp"

namespace core::utils {
namespace {
static const std::map<EventLevel, std::string> kEventLevelNameMap = {
  {EventLevel::DEBUG, "DEBUG"},  //
  {EventLevel::ERROR, "ERROR"},  //
  {EventLevel::FATAL, "FATAL"},  //
  {EventLevel::INFO, "INFO"},    //
  {EventLevel::WARN, "WARN"}     //
};
}  // namespace

const std::string& EventLevelToString(const EventLevel event) {
  // TODO(bara): find a way to enforce map to have all entree of EventLevel then
  // you can replace the replace this function by just:
  // return kEventLevelNameMap.at(event);
  auto it = kEventLevelNameMap.find(event);
  if (it == kEventLevelNameMap.end()) {
    throw Exception("Undefined EventLevel.");
  }
  return it->second;
}

bool IsCritical(const EventLevel event) {
  return event == EventLevel::ERROR || event == EventLevel::FATAL;
}

std::ostream& operator<<(std::ostream& os, const EventLevel event) {
  return os << EventLevelToString(event);
}

}  // namespace core::utils
