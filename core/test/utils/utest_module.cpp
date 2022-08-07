#include <gtest/gtest.h>

#include "core/utils/module.hpp"

using namespace core::utils;

constexpr auto UndefinedType = core::utils::ModuleType::UNDEFINED;

TEST(ModuleType, ToString)
{
  EXPECT_EQ("SENSOR", ModuleTypeToString(ModuleType::SENSOR));
  EXPECT_EQ("UNDEFINED", ModuleTypeToString(ModuleType::UNDEFINED));
}

TEST(ModuleType, Type)
{
  const ModuleAbs mod(UndefinedType);
  EXPECT_EQ(UndefinedType, mod.Type());
}

TEST(ModuleType, Name)
{
  const ModuleAbs mod(UndefinedType);
  EXPECT_EQ("UNDEFINED-Unnamed", mod.Name());
}