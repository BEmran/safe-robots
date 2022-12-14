// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/data.hpp"
#include "core/utils/data_logger.hpp"
#include "core/utils/formatter2.hpp"
#include "core/utils/subject.hpp"
#include "core/utils/writer.hpp"
#include "utest/utils.hpp"

using core::utils::DataLogger;
using core::utils::Formatter;
using core::utils::HeadingData;
using core::utils::InfoLabeledModifier;
using core::utils::Logger;
using core::utils::LoggerConfig;
using core::utils::Subject;
using core::utils::TemperatureData;
using core::utils::Writer;
using core::utils::WriterFormatterPair;

Formatter kFormatter;

std::shared_ptr<Logger> CreateLogger(std::stringstream& ss) {
  auto writer = std::make_shared<Writer>(ss);
  LoggerConfig config;
  config.wf_pairs = {{writer, kFormatter}};
  return std::make_shared<Logger>(config);
}

TEST(MockLogger, LogWithDefaultHeaderAndLM) {
  auto sub = std::make_shared<Subject<HeadingData>>("node");

  HeadingData heading;
  heading.value = 1.0;
  sub->SetAndNotify(heading);

  std::stringstream ss;
  auto logger = CreateLogger(ss);
  DataLogger data_logger(logger);

  data_logger.Observe(sub);
  data_logger.Log();
  auto expect1 = ExpectFormattedLoggerMsg(&kFormatter, InfoLabeledModifier(),
                                          "", "1.000000");
  EXPECT_EQ(expect1, ss.str());
  ss.str("");

  heading.value = 2.0;
  sub->SetAndNotify(heading);
  data_logger.Log();
  auto expect2 = ExpectFormattedLoggerMsg(&kFormatter, InfoLabeledModifier(),
                                          "", "2.000000");
  EXPECT_EQ(expect2, ss.str());
  ss.str("");
}

TEST(MockLogger, Log2DataWithDefaultHeaderAndLM) {
  auto sub1 = std::make_shared<Subject<HeadingData>>("node1");
  HeadingData heading;
  heading.value = 1.0;
  sub1->SetAndNotify(heading);

  auto sub2 = std::make_shared<Subject<TemperatureData>>("node2");
  TemperatureData temp;
  temp.value = -10.0;
  sub2->SetAndNotify(temp);

  std::stringstream ss;
  auto logger = CreateLogger(ss);
  DataLogger data_logger(logger);

  data_logger.Observe(sub1);
  data_logger.Observe(sub2);
  data_logger.Log();
  auto expect = ExpectFormattedLoggerMsg(&kFormatter, InfoLabeledModifier(), "",
                                         "1.000000, -10.000000");
  EXPECT_EQ(expect, ss.str());
}
