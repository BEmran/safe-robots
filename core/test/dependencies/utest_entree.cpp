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
  EXPECT_EQ(NodeTypeToString(NodeType::LIST), "LIST");
  EXPECT_EQ(NodeTypeToString(NodeType::MAP), "MAP");
  EXPECT_EQ(NodeTypeToString(NodeType::SINGLE), "SINGLE");
  EXPECT_EQ(NodeTypeToString(NodeType::SEQUENCE), "SEQUENCE");
  EXPECT_EQ(NodeTypeToString(NodeType::UNDEFINED), "UNDEFINED");
}

const std::string KEY = "Key";

// Single ---------------------------------------------------------------------
const std::string single_value = "Value";

TEST(Single, Key) {
  yaml::Single single(KEY, EntreeType::STRING, single_value);
  EXPECT_EQ(KEY, single.Key());
}

TEST(Single, Type) {
  yaml::Single single(KEY, EntreeType::STRING, single_value);
  EXPECT_EQ(NodeType::SINGLE, single.Type());
}

TEST(Single, GetEntreeType) {
  yaml::Single single(KEY, EntreeType::STRING, single_value);
  EXPECT_EQ(EntreeType::STRING, single.GetEntreeType());
}

TEST(Single, GetEntreeValue) {
  yaml::Single single(KEY, EntreeType::STRING, single_value);
  EXPECT_EQ(single_value, single.GetEntreeValue());
}

TEST(Single, Print) {
  yaml::Single single(KEY, yaml::EntreeType::STRING, single_value);
  const std::string actual = single.Print("");
  const std::string expect = KEY + ": " + single_value;
  EXPECT_EQ(expect, actual);
}

// Sequence -------------------------------------------------------------------
const std::vector<std::string> list_value{"1", "2", "3", "4"};

TEST(Sequence, Key) {
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(KEY, list.Key());
}

TEST(Sequence, Type) {
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(NodeType::SEQUENCE, list.Type());
}

TEST(Sequence, GetEntreeType) {
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  EXPECT_EQ(yaml::EntreeType::STRING, list.GetEntreeType());
}

TEST(Sequence, GetEntreeValues) {
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  const auto actual = list.GetEntreeValue();
  const auto result =
    std::equal(list_value.begin(), list_value.end(), actual.begin(),
               [](const auto str1, const auto str2) { return str1 == str2; });
  EXPECT_TRUE(result);
}

TEST(Sequence, Print) {
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  const std::string expect = list.Print("");
  std::string actual = KEY + ": [";
  for (size_t i{0}; i < list_value.size(); i++) {
    actual += list_value[i] + " ";
  }
  actual.back() = ']';
  EXPECT_EQ(expect, actual);
}

// Map ------------------------------------------------------------------
TEST(Map, Type) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Single single2(KEY, EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  EXPECT_EQ(NodeType::MAP, map.Type());
}

TEST(Map, NotValidNodeMap) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Single single2(KEY, EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  EXPECT_FALSE(map.IsNodeMapValid());
}

TEST(Map, ValidNodeMap) {
  yaml::Single single1(KEY + "1", EntreeType::STRING, single_value);
  yaml::Single single2(KEY + "2", EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  EXPECT_TRUE(map.IsNodeMapValid());
}

TEST(Map, FailedToFindKey) {
  const auto unkown_key = "unknown";
  yaml::Single single1(KEY + "1", EntreeType::STRING, single_value);
  yaml::Single single2(KEY + "2", EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  auto result = map[unkown_key];
  EXPECT_FALSE(result);
}

TEST(Map, FindKey) {
  yaml::Single single1(KEY + "1", EntreeType::STRING, single_value);
  yaml::Single single2(KEY + "2", EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  auto result = map[single1.Key()];
  ASSERT_TRUE(result);
  EXPECT_EQ(map.GetEntreeValue()[0], result.value());
}

TEST(Map, FindFirstKeyEntree) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Single single2(KEY, EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  auto result = map[single1.Key()];
  ASSERT_TRUE(result);
  ASSERT_FALSE(map.IsNodeMapValid());
  EXPECT_EQ(map.GetEntreeValue()[0], result.value());
}

TEST(Map, FindParent) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Single single2(KEY, EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  auto result = map.FindParent(single1.Key());
  ASSERT_TRUE(result);
  EXPECT_EQ(&map, result.value());
}

TEST(Map, FailedToFindParent) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Single single2(KEY, EntreeType::STRING, single_value);
  yaml::Map map(KEY, {&single1, &single2});
  auto result = map.FindParent("unknown");
  ASSERT_FALSE(result);
}

TEST(Cash, FindParentOfNestedMap) {
  yaml::Single single1("key2", yaml::EntreeType::STRING, "str");
  yaml::Single single2("key3", yaml::EntreeType::STRING, "123");
  std::vector<yaml::Node*> inner_struct_vec2{&single1, &single2};
  yaml::Map inner_map("key4", inner_struct_vec2);
  yaml::Single single3("key5", yaml::EntreeType::STRING, "char");
  yaml::Single single4("key6", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> outer_struct_vec1{&single3, &single4, &inner_map};
  yaml::Map outer_map("key1", outer_struct_vec1);
  auto result = outer_map.FindParent(single1.Key());
  ASSERT_TRUE(result);
  EXPECT_EQ(&inner_map, result.value());
}

TEST(Map, Print) {
  yaml::Single single1(KEY, EntreeType::STRING, single_value);
  yaml::Sequence list(KEY, yaml::EntreeType::STRING, list_value);
  yaml::Map map1(KEY, {&single1, &list});
  yaml::Map map2(KEY, {&map1, &single1, &single1});
  std::cout << map2.Print("") << std::endl;
  EXPECT_TRUE(true);
}

// List -----------------------------------------------------------------------
TEST(List, Type) {
  yaml::Single single1("key1", yaml::EntreeType::STRING, "abc");
  yaml::Single single2("key2", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key1", yaml::EntreeType::STRING, "def");
  yaml::Single single4("key2", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> struct_vec1{&single1, &single2};
  std::vector<yaml::Node*> struct_vec2{&single3, &single4};
  yaml::Map map1(struct_vec1);
  yaml::Map map2(struct_vec2);
  std::vector<yaml::Node*> list_vec{&map1, &map2};
  yaml::List list("key3", list_vec);
  EXPECT_EQ(NodeType::LIST, list.Type());
  EXPECT_EQ(NodeType::MAP, list.GetEntreeValue()[0]->Type());
  EXPECT_EQ(NodeType::MAP, list.GetEntreeValue()[1]->Type());
}

TEST(List, Print) {
  yaml::Single single1("key1", yaml::EntreeType::STRING, "abc");
  yaml::Single single2("key2", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key1", yaml::EntreeType::STRING, "def");
  yaml::Single single4("key2", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> struct_vec1{&single1, &single2};
  std::vector<yaml::Node*> struct_vec2{&single3, &single4};
  yaml::Map map1(struct_vec1);
  yaml::Map map2(struct_vec2);
  std::vector<yaml::Node*> list_vec{&map1, &map2};
  yaml::List list("key3", list_vec);
  std::cout << list.Print("") << std::endl;
  EXPECT_TRUE(true);
}
