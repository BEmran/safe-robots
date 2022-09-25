// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <dependencies/entree.hpp>
#include <fstream>

using yaml::EntreeType;
using yaml::EntreeTypeToString;
using yaml::NodeType;
using yaml::NodeTypeToString;

TEST(EntreeTypeStringMap, Test) {
  EXPECT_EQ(EntreeTypeToString(EntreeType::DECIMAL), "DECIMAL");
  EXPECT_EQ(EntreeTypeToString(EntreeType::REAL), "REAL");
  EXPECT_EQ(EntreeTypeToString(EntreeType::STRING), "STRING");
  EXPECT_EQ(EntreeTypeToString(EntreeType::UNDEFINED), "UNDEFINED");
}

TEST(NodeTypeToString, Test) {
  EXPECT_EQ(NodeTypeToString(NodeType::STRUCT), "STRUCT");
  EXPECT_EQ(NodeTypeToString(NodeType::SINGLE), "SINGLE");
  EXPECT_EQ(NodeTypeToString(NodeType::LIST), "LIST");
  EXPECT_EQ(NodeTypeToString(NodeType::UNDEFINED), "UNDEFINED");
}

const std::string key = "Key";

// Single ---------------------------------------------------------------------
const std::string single_value = "Value";

TEST(Single, Key) {
  yaml::Single single(key, EntreeType::STRING, single_value);
  EXPECT_EQ(key, single.Key());
}

TEST(Single, Type) {
  yaml::Single single(key, EntreeType::STRING, single_value);
  EXPECT_EQ(NodeType::SINGLE, single.Type());
}

TEST(Single, GetEntreeType) {
  yaml::Single single(key, EntreeType::STRING, single_value);
  EXPECT_EQ(EntreeType::STRING, single.GetEntreeType());
}

TEST(Single, GetEntreeValue) {
  yaml::Single single(key, EntreeType::STRING, single_value);
  EXPECT_EQ(single_value, single.GetEntreeValue());
}

TEST(Single, ToString) {
  yaml::Single single(key, yaml::EntreeType::STRING, single_value);
  const std::string actual = single.ToString();
  const std::string expect = key + "[SINGLE:STRING]: " + single_value;
  EXPECT_EQ(expect, actual);
}

// Vector ---------------------------------------------------------------------
const std::vector<std::string> list_value{"1", "2", "3", "4"};

TEST(List, Key) {
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(key, list.Key());
}

TEST(List, Type) {
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(NodeType::LIST, list.Type());
}

TEST(List, GetEntreeType) {
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(yaml::EntreeType::STRING, list.GetEntreeType());
}

TEST(List, GetEntreeValues) {
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  const auto actual = list.GetEntreeValues();
  const auto result =
    std::equal(list_value.begin(), list_value.end(), actual.begin(),
               [](const auto str1, const auto str2) { return str1 == str2; });
  EXPECT_TRUE(result);
}

TEST(List, ToString) {
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  const std::string expect = list.ToString();
  std::string actual = key + "[LIST:STRING]: [";
  for (size_t i{0}; i < list_value.size(); i++) {
    actual += list_value[i];
    if (i < list_value.size() - 1)
      actual += ", ";
  }
  actual += "]";
  EXPECT_EQ(expect, actual);
}

TEST(Structure, Type) {
  yaml::Single single1(key, EntreeType::STRING, single_value);
  yaml::Single single2(key, EntreeType::STRING, single_value);
  yaml::Structure structure(key, {&single1, &single2});
  EXPECT_EQ(NodeType::STRUCT, structure.Type());
}

TEST(Structure, ToString) {
  yaml::Single single1(key, EntreeType::STRING, single_value);
  yaml::List list(key, yaml::EntreeType::STRING, list_value);
  yaml::Structure structure1(key, {&single1, &list});
  yaml::Structure structure2(key, {&structure1, &single1, &single1});
  std::cout << structure2.ToString() << std::endl;
  EXPECT_TRUE(true);
}