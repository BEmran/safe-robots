// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/data.hpp"
#include "core/utils/subject.hpp"
#include "utest/utils_data.hpp"

using core::utils::HeadingData;
using core::utils::ObserverCallback;
using core::utils::Subject;

class MockObserver {
 public:
  void CB(const int& n) {
    value = n;
  }
  int value = 0;
};

TEST(Subject, RegisterMemberFunction) {
  MockObserver obs;
  auto cb = std::make_shared<ObserverCallback<int>>(
    std::bind(&MockObserver::CB, &obs, std::placeholders::_1));

  Subject<int> sub("sub");
  sub.Register(cb);

  int updated_value = 1.0;
  sub.Set(updated_value);

  EXPECT_EQ(updated_value, sub.Get());
  EXPECT_EQ(updated_value, obs.value);
}

TEST(Subject, UnregisterMemberFunction) {
  MockObserver obs;
  auto cb = std::make_shared<ObserverCallback<int>>(
    std::bind(&MockObserver::CB, &obs, std::placeholders::_1));

  Subject<int> sub("sub");
  sub.Register(cb);

  int updated_value = 1.0;
  sub.Set(updated_value);

  EXPECT_EQ(updated_value, sub.Get());
  EXPECT_EQ(updated_value, obs.value);

  sub.Unregister(cb);
  updated_value = 2.0;
  sub.Set(updated_value);

  EXPECT_EQ(updated_value, sub.Get());
  EXPECT_NE(updated_value, obs.value);
}

TEST(Subject, RegisterLambda) {
  HeadingData data;
  auto lambda = [&data](const HeadingData& d) { data = d; };

  Subject<HeadingData> sub("sub");
  auto cb = std::make_shared<ObserverCallback<HeadingData>>(lambda);
  sub.Register(cb);

  HeadingData updated_data;
  updated_data.value = 1.0;
  sub.Set(updated_data);

  ExpectDoubleDataEq(updated_data, sub.Get());
  ExpectDoubleDataEq(updated_data, data);
}

TEST(Subject, UnregisterLambda) {
  HeadingData data;
  auto lambda = [&data](const HeadingData& d) { data = d; };

  Subject<HeadingData> sub("sub");
  auto cb = std::make_shared<ObserverCallback<HeadingData>>(lambda);

  sub.Register(cb);
  HeadingData updated_data;
  updated_data.value = 1.0;
  sub.Set(updated_data);
  ExpectDoubleDataEq(updated_data, sub.Get());
  ExpectDoubleDataEq(updated_data, data);

  sub.Unregister(cb);
  updated_data.value = 2.0;
  sub.Set(updated_data);
  ExpectDoubleDataEq(updated_data, sub.Get());
  EXPECT_NE(updated_data.value, data.value);
}
