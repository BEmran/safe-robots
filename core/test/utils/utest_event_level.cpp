// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/event_level.hpp"
#include "utest/utils.hpp"

using core::utils::IsCritical;

// test converting an EventLevel to string
TEST(EventLevel, EventLevelToString) {
  for (size_t i = 0; i < kEvents.size(); i++) {
    EXPECT_EQ(kLabels[i], EventLevelToString(kEvents[i]));
  }
}

// test stream EventLevel should be the same as EventLevelToString function
TEST(EventLevel, StreamName) {
  for (const auto& event : kEvents) {
    std::stringstream ss;
    ss << event;
    EXPECT_EQ(EventLevelToString(event), ss.str());
  }
}

// test check if event is critical
TEST(EventLevel, IsCritical) {
  EXPECT_FALSE(IsCritical(EventLevel::DEBUG));
  EXPECT_FALSE(IsCritical(EventLevel::INFO));
  EXPECT_FALSE(IsCritical(EventLevel::WARN));
  EXPECT_TRUE(IsCritical(EventLevel::ERROR));
  EXPECT_TRUE(IsCritical(EventLevel::FATAL));
}
