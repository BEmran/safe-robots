// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/event_level.hpp"
#include "utest/utils.hpp"

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
