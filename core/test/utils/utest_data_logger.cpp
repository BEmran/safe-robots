// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/data.hpp"
#include "core/utils/data_logger.hpp"
#include "core/utils/formatter2.hpp"
#include "core/utils/subject.hpp"
#include "core/utils/writer.hpp"
#include "utest/utils.hpp"

using core::utils::DataLogger;
using core::utils::HeadingData;
using core::utils::Logger;
using core::utils::NullFormater;
using core::utils::Subject;
using core::utils::WriterFormatterPair;

class MockWriter : public core::utils::Writer {
 public:
  void Dump(const std::string& str) const {
    msg_ = str;
    std::cout << "writer: " << msg_ << std::endl;
  }
  std::string Msg() const {
    return msg_;
  }

 private:
  mutable std::string msg_;
};

TEST(MockLogger, LogWithDefaultHeaderAndLM) {
  auto writer = std::make_shared<MockWriter>();
  auto formater = std::make_shared<NullFormater>();

  std::vector<WriterFormatterPair> writer_formatter_vec{{writer, formater}};

  auto mock_logger = std::make_shared<Logger>(writer_formatter_vec);

  auto sub = std::make_shared<Subject<HeadingData>>("sub");

  HeadingData heading;
  heading.value = 1.0;
  sub->Set(heading);

  DataLogger data_logger(mock_logger);

  data_logger.Observe(sub);
  data_logger.Log();

  sub->Inform();
  heading.value = 2.0;
  sub->Set(heading);
  data_logger.Log();

  heading.value = 3.0;
  sub->Set(heading);
  data_logger.Log();

  EXPECT_EQ(sub->Get().ToString(), writer->Msg());
}
