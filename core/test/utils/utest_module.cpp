#include <gtest/gtest.h>

#include "core/utils/module.hpp"

using namespace core::utils;

constexpr auto UndefinedType = core::utils::ModuleType::UNDEFINED;
constexpr auto Name = "name";

TEST(ModuleType, ToString)
{
  EXPECT_EQ("SENSOR", ModuleTypeToString(ModuleType::SENSOR));
  EXPECT_EQ("UNDEFINED", ModuleTypeToString(ModuleType::UNDEFINED));
}

TEST(ModuleType, Type)
{
  const ModuleAbs mod(UndefinedType, Name);
  EXPECT_EQ(UndefinedType, mod.Type());
}

TEST(ModuleType, Name)
{
  const ModuleAbs mod(UndefinedType, Name);
  EXPECT_EQ(Name, mod.Name());
}