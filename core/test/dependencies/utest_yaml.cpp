// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <dependencies/yaml.hpp>
#include <fstream>
#include <iostream>

void ExpectEqStructure(yaml::Node* node1, yaml::Node* node2);
void ExpectEqList(yaml::Node* node1, yaml::Node* node2);
void ExpectEqSequence(yaml::Node* node1, yaml::Node* node2);
void ExpectEqSingle(yaml::Node* node1, yaml::Node* node2);

void ExpectEqVector(std::vector<std::string> vec1,
                    std::vector<std::string> vec2) {
  EXPECT_EQ(vec1.size(), vec2.size());
  for (size_t idx = 0; idx < vec1.size(); ++idx) {
    EXPECT_EQ(vec1[idx], vec2[idx]);
  }
}

void ExpectEqSingle(yaml::Node* node1, yaml::Node* node2) {
  ASSERT_EQ(yaml::NodeType::SINGLE, node1->Type());
  ASSERT_EQ(yaml::NodeType::SINGLE, node2->Type());
  auto single1 = dynamic_cast<yaml::Single*>(node1);
  auto single2 = dynamic_cast<yaml::Single*>(node2);
  auto str1 = single1->GetEntreeValue();
  auto str2 = single2->GetEntreeValue();
  EXPECT_EQ(str1, str2);
  EXPECT_EQ(single1->Print(), single2->Print());
  EXPECT_EQ(single1->Key(), single2->Key());
}

void ExpectEqSequence(yaml::Node* node1, yaml::Node* node2) {
  ASSERT_EQ(yaml::NodeType::SEQUENCE, node1->Type());
  ASSERT_EQ(yaml::NodeType::SEQUENCE, node2->Type());
  auto seq1 = dynamic_cast<yaml::Sequence*>(node1);
  auto seq2 = dynamic_cast<yaml::Sequence*>(node2);
  auto vec1 = seq1->GetEntreeValue();
  auto vec2 = seq2->GetEntreeValue();
  ExpectEqVector(vec1, vec2);
  EXPECT_EQ(seq1->Print(), seq2->Print());
  EXPECT_EQ(seq1->Key(), seq2->Key());
}

void ExpectEqList(yaml::Node* node1, yaml::Node* node2) {
  ASSERT_EQ(yaml::NodeType::LIST, node1->Type());
  ASSERT_EQ(yaml::NodeType::LIST, node2->Type());
  auto list1 = dynamic_cast<yaml::List*>(node1);
  auto list2 = dynamic_cast<yaml::List*>(node2);
  auto vec1 = list1->GetEntreeValue();
  auto vec2 = list2->GetEntreeValue();
  ASSERT_EQ(vec1.size(), vec2.size());
  for (size_t idx = 0; idx < vec1.size(); ++idx) {
    EXPECT_EQ(vec1[idx]->Type(), vec2[idx]->Type());
    switch (vec1[idx]->Type()) {
      case yaml::NodeType::SINGLE:
        ExpectEqSingle(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::SEQUENCE:
        ExpectEqSequence(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::MAP:
        ExpectEqStructure(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::LIST:
        ExpectEqList(vec1[idx], vec2[idx]);
        break;
      default:
        std::cerr << "undefined type";
    }
  }
}

void ExpectEqStructure(yaml::Node* node1, yaml::Node* node2) {
  ASSERT_EQ(yaml::NodeType::MAP, node1->Type());
  ASSERT_EQ(yaml::NodeType::MAP, node2->Type());
  auto struct1 = dynamic_cast<yaml::Map*>(node1);
  auto struct2 = dynamic_cast<yaml::Map*>(node2);
  auto vec1 = struct1->GetEntreeValue();
  auto vec2 = struct2->GetEntreeValue();
  ASSERT_EQ(vec1.size(), vec2.size());
  for (size_t idx = 0; idx < vec1.size(); ++idx) {
    EXPECT_EQ(vec1[idx]->Type(), vec2[idx]->Type());
    switch (vec1[idx]->Type()) {
      case yaml::NodeType::SINGLE:
        ExpectEqSingle(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::SEQUENCE:
        ExpectEqSequence(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::MAP:
        ExpectEqStructure(vec1[idx], vec2[idx]);
        break;
      case yaml::NodeType::LIST:
        ExpectEqList(vec1[idx], vec2[idx]);
        break;
      default:
        std::cerr << "undefined type";
    }
  }
  EXPECT_EQ(node1->Print(), node2->Print());
}

TEST(Cash, ExtractSingle) {
  std::string config = "{key1: 123}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << std::endl;

  yaml::Single single("key1", yaml::EntreeType::STRING, "123");
  std::vector<yaml::Node*> expect_vec{&single};
  yaml::Map expect("", expect_vec);

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, Extract2Singles) {
  std::string config = "{key1: 123, key2: str}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << std::endl;

  yaml::Single single1("key1", yaml::EntreeType::STRING, "123");
  yaml::Single single2("key2", yaml::EntreeType::STRING, "str");
  std::vector<yaml::Node*> expect_vec{&single1, &single2};
  yaml::Map expect("", expect_vec);

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, ExtractSequence) {
  std::string config = "{key1: [1 2 3]}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << std::endl;

  yaml::Sequence seq("key1", yaml::EntreeType::STRING, {"1", "2", "3"});
  std::vector<yaml::Node*> expect_vec{&seq};
  yaml::Map expect("", expect_vec);

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, ExtractStruct) {
  std::string config = "{key1: {key2: str, key3: 123}}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << std::endl;

  yaml::Single single1("key2", yaml::EntreeType::STRING, "str");
  yaml::Single single2("key3", yaml::EntreeType::STRING, "123");
  std::vector<yaml::Node*> struct_vec{&single1, &single2};
  yaml::Map map("key1", struct_vec);
  yaml::Map expect("", {&map});

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, ExtractNestedStruct) {
  std::string config =
    "{key1: {key2: str, key3: 123, key4: {key5: char, key6: 456}}}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << std::endl;

  yaml::Single single1("key2", yaml::EntreeType::STRING, "str");
  yaml::Single single2("key3", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key5", yaml::EntreeType::STRING, "char");
  yaml::Single single4("key6", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> inner_struct_vec2{&single3, &single4};
  yaml::Map inner_struct("key4", inner_struct_vec2);
  std::vector<yaml::Node*> outer_struct_vec1{&single1, &single2, &inner_struct};
  yaml::Map outer_struct("key1", outer_struct_vec1);
  yaml::Map expect("", {&outer_struct});

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, ReEvaluated) {
  std::string config =
    "{key1: ["
    "{key2: abc, key3: 123},"
    "{key2: def, key3: 456}"
    "]}";
  auto actual = yaml::LoadConfig(config);
  auto re_evaluated = yaml::LoadConfig(actual->Print());
  ExpectEqStructure(re_evaluated, actual);
}

TEST(Cash, ExtractList) {
  std::string config =
    "{key1: ["
    "{key2: abc, key3: 123},"
    "{key2: def, key3: 456}"
    "]}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << "\n" << std::endl;

  yaml::Single single1("key2", yaml::EntreeType::STRING, "abc");
  yaml::Single single2("key3", yaml::EntreeType::STRING, "123");
  yaml::Single single3("key2", yaml::EntreeType::STRING, "def");
  yaml::Single single4("key3", yaml::EntreeType::STRING, "456");
  std::vector<yaml::Node*> struct_vec1{&single1, &single2};
  std::vector<yaml::Node*> struct_vec2{&single3, &single4};
  yaml::Map map1(struct_vec1);
  yaml::Map map2(struct_vec2);
  std::vector<yaml::Node*> list_vec{&map1, &map2};
  yaml::List list("key1", list_vec);
  yaml::Map expect({&list});
  std::cout << expect.Print() << std::endl;

  ExpectEqStructure(&expect, actual);
}

TEST(Cash, ExtractNestedList) {
  std::string config =
    "{key1: ["
    "{key2: [a b c], key3: [{key4: a, key5: 1}, {key4: b, key5: 2}]},"
    "{key2: [e f g], key3: [{key4: x, key5: 3}, {key4: y, key5: 4}]}"
    "]}";
  auto actual = yaml::LoadConfig(config);
  std::cout << actual->Print() << "\n" << std::endl;

  yaml::Single single111("key4", yaml::EntreeType::STRING, "a");
  yaml::Single single112("key5", yaml::EntreeType::STRING, "1");
  yaml::Map struct11({&single111, &single112});
  yaml::Single single121("key4", yaml::EntreeType::STRING, "b");
  yaml::Single single122("key5", yaml::EntreeType::STRING, "2");
  yaml::Map struct12({&single121, &single122});
  yaml::List list1("key3", {&struct11, &struct12});
  yaml::Sequence seq1("key2", yaml::EntreeType::STRING, {"a", "b", "c"});
  yaml::Map struct1({&seq1, &list1});

  yaml::Single single211("key4", yaml::EntreeType::STRING, "x");
  yaml::Single single212("key5", yaml::EntreeType::STRING, "3");
  yaml::Map struct21({&single211, &single212});
  yaml::Single single221("key4", yaml::EntreeType::STRING, "y");
  yaml::Single single222("key5", yaml::EntreeType::STRING, "4");
  yaml::Map struct22({&single221, &single222});
  yaml::List list2("key3", {&struct21, &struct22});
  yaml::Sequence seq2("key2", yaml::EntreeType::STRING, {"e", "f", "g"});
  yaml::Map struct2({&seq2, &list2});

  yaml::List list("key1", {&struct1, &struct2});
  yaml::Map expect({&list});
  std::cout << expect.Print() << std::endl;

  ExpectEqStructure(&expect, actual);
}
