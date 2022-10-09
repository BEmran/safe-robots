// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <iostream>
#include <map>
#include <string>

#include "core/utils/exception.hpp"
#include "core/utils/labeld_modifier.hpp"

namespace core::utils {
namespace {
// static const std::map<EventLevel,string_view> kEventLevelNameMap = {
//   {EventLevel::ALL, "ALL"},            //
//   {EventLevel::CRITICAL, "CRITICAL"},  //
//   {EventLevel::DEBUG, "DEBUG"},        //
//   {EventLevel::ERROR, "ERROR"},        //
//   {EventLevel::INFO, "INFO"},          //
//   {EventLevel::TRACE, "TRACE"},        //
//   {EventLevel::WARN, "WARN"}};         //
// }  // namespace
static const std::map<EventLevel, std::string_view> kEventLevelNameMap = {
  {EventLevel::INFO, "INFO"},     //
  {EventLevel::DEBUG, "DEBUG"},   //
  {EventLevel::WARN, "WARN"},     //
  {EventLevel::ERROR, "ERROR"}};  //
}  // namespace

std::string EventLevelToString(const EventLevel event) {
  auto it = kEventLevelNameMap.find(event);
  if (it == kEventLevelNameMap.end()) {
    throw Exception("Undefined event level.");
  }
  return it->second.data();
}

}  // namespace core::utils
