// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/module.hpp"

using core::utils::ModuleAbs;
using core::utils::ModuleType;

constexpr auto UndefinedType = ModuleType::UNDEFINED;
constexpr auto Name = "name";
constexpr auto Debug = false;

TEST(ModuleType, ToString) {
  EXPECT_EQ("SENSOR", ModuleTypeToString(ModuleType::SENSOR));
  EXPECT_EQ("UNDEFINED", ModuleTypeToString(ModuleType::UNDEFINED));
}

TEST(ModuleType, Type) {
  const ModuleAbs mod(UndefinedType, Name, Debug);
  EXPECT_EQ(UndefinedType, mod.Type());
}

TEST(ModuleType, Name) {
  const ModuleAbs mod(UndefinedType, Name, Debug);
  EXPECT_EQ(Name, mod.Name());
}

TEST(ModuleType, EnableDebug) {
  const ModuleAbs mod(UndefinedType, Name, true);
  EXPECT_EQ(true, mod.IsDebugEnabled());
}

TEST(ModuleType, DisableDebug) {
  const ModuleAbs mod(UndefinedType, Name, false);
  EXPECT_EQ(false, mod.IsDebugEnabled());
}
