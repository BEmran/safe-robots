// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <dependencies/entree.hpp>
#include <fstream>

using yaml::EntreeType;
using yaml::EntreeTypeToString;
using yaml::NodeType;
using yaml::NodeTypeToString;

TEST(EntreeTypeStringMap, Test) {
  EXPECT_EQ(EntreeTypeToString(EntreeType::INTEGER), "INTEGER");
  EXPECT_EQ(EntreeTypeToString(EntreeType::FLOAT), "FLOAT");
  EXPECT_EQ(EntreeTypeToString(EntreeType::STRING), "STRING");
  EXPECT_EQ(EntreeTypeToString(EntreeType::UNDEFINED), "UNDEFINED");
}

TEST(NodeTypeToString, Test) {
  EXPECT_EQ(NodeTypeToString(NodeType::SEQUENCE), "SEQUENCE");
  EXPECT_EQ(NodeTypeToString(NodeType::SINGLE), "SINGLE");
  EXPECT_EQ(NodeTypeToString(NodeType::STRUCT), "STRUCT");
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

TEST(Single, Print) {
  yaml::Single single(key, yaml::EntreeType::STRING, single_value);
  const std::string actual = single.Print("");
  // const std::string expect = key + "[SINGLE:STRING]: " + single_value;
  const std::string expect = key + ": " + single_value;
  EXPECT_EQ(expect, actual);
}

// Vector ---------------------------------------------------------------------
const std::vector<std::string> list_value{"1", "2", "3", "4"};

TEST(Sequence, Key) {
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(key, list.Key());
}

TEST(Sequence, Type) {
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(NodeType::SEQUENCE, list.Type());
}

TEST(Sequence, GetEntreeType) {
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(yaml::EntreeType::STRING, list.GetEntreeType());
}

TEST(Sequence, GetEntreeValues) {
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  const auto actual = list.GetEntreeValue();
  const auto result =
    std::equal(list_value.begin(), list_value.end(), actual.begin(),
               [](const auto str1, const auto str2) { return str1 == str2; });
  EXPECT_TRUE(result);
}

TEST(Sequence, Print) {
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  const std::string expect = list.Print("");
  // std::string actual = key + "[SEQUENCE:STRING]: [";
  std::string actual = key + ": [";
  for (size_t i{0}; i < list_value.size(); i++) {
    actual += list_value[i] + " ";
  }
  actual.back() = ']';
  EXPECT_EQ(expect, actual);
}

TEST(Structure, Type) {
  yaml::Single single1(key, EntreeType::STRING, single_value);
  yaml::Single single2(key, EntreeType::STRING, single_value);
  yaml::Structure structure(key, {&single1, &single2});
  EXPECT_EQ(NodeType::STRUCT, structure.Type());
}

TEST(Structure, Print) {
  yaml::Single single1(key, EntreeType::STRING, single_value);
  yaml::Sequence list(key, yaml::EntreeType::STRING, list_value);
  yaml::Structure structure1(key, {&single1, &list});
  yaml::Structure structure2(key, {&structure1, &single1, &single1});
  std::cout << structure2.Print("") << std::endl;
  EXPECT_TRUE(true);
}

TEST(List, Type) {
  yaml::Single single1("key1", yaml::EntreeType::STRING, "abc");
  yaml::Single single2("key2", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key1", yaml::EntreeType::STRING, "def");
  yaml::Single single4("key2", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> struct_vec1{&single1, &single2};
  std::vector<yaml::Node*> struct_vec2{&single3, &single4};
  yaml::Structure structure1(struct_vec1);
  yaml::Structure structure2(struct_vec2);
  std::vector<yaml::Node*> list_vec{&structure1, &structure2};
  yaml::List list("key3", list_vec);
  EXPECT_EQ(NodeType::LIST, list.Type());
  EXPECT_EQ(NodeType::STRUCT, list.GetEntreeValues()[0]->Type());
  EXPECT_EQ(NodeType::STRUCT, list.GetEntreeValues()[1]->Type());
}

TEST(List, Print) {
  yaml::Single single1("key1", yaml::EntreeType::STRING, "abc");
  yaml::Single single2("key2", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key1", yaml::EntreeType::STRING, "def");
  yaml::Single single4("key2", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> struct_vec1{&single1, &single2};
  std::vector<yaml::Node*> struct_vec2{&single3, &single4};
  yaml::Structure structure1(struct_vec1);
  yaml::Structure structure2(struct_vec2);
  std::vector<yaml::Node*> list_vec{&structure1, &structure2};
  yaml::List list("key3", list_vec);
  std::cout << list.Print("") << std::endl;
  EXPECT_TRUE(true);
}
