#include "core/utils/module.hpp"

namespace core::utils
{
using namespace std::literals::string_literals;  // to use s literals

std::string ModuleTypeToString(const ModuleType type)
{
  switch (type)
  {
    case ModuleType::SENSOR:
      return "SENSOR"s;
    default:
      return "UNDEFINED"s;
  }
}

ModuleAbs::ModuleAbs(const ModuleType type, const std::string& name,
                     const bool debug)
  : type_{type}, name_{name}, debug_{debug}
{
}

ModuleType ModuleAbs::Type() const
{
  return type_;
}

std::string ModuleAbs::Name() const
{
  return name_;
}

bool ModuleAbs::IsDebugEnabled() const
{
  return debug_;
}
}  // namespace core::utils