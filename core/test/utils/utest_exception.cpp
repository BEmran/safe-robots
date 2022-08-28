#include <gtest/gtest.h>

#include "core/utils/exception.hpp"

using namespace core::utils;

TEST(Exception, ThrowException) {
  auto throw_exception = []() { throw Exception(""); };
  EXPECT_THROW(throw_exception(), Exception);
}

TEST(Exception, ThrowExceptionWithMsg) {
  std::string msg = "error msg";
  std::string caught_msg;
  try {
    throw Exception(msg);
  } catch (Exception& e) {
    caught_msg = e.what();
  }
  EXPECT_EQ(msg, caught_msg);
}

TEST(ExceptionFactory, ThrowExceptionWithHeader) {
  std::string header = "exception header";
  ExceptionFactory except_factory(header);
  auto throw_exception = [&except_factory]() { except_factory.Throw(""); };
  EXPECT_THROW(throw_exception(), Exception);
}

TEST(ExceptionFactory, ThrowExceptionWithHeaderAndMsg) {
  std::string header = "exception header";
  ExceptionFactory except_factory(header);
  std::string msg = "error msg";
  std::string caught_msg;
  try {
    except_factory.Throw(msg);
  } catch (Exception& e) {
    caught_msg = e.what();
  }
  EXPECT_EQ(header + ": " + msg, caught_msg);
}

TEST(ExceptionFactory, ThrowExceptionWithMsgButNoHeader) {
  std::string header;
  ExceptionFactory except_factory(header);
  std::string msg = "error msg";
  std::string caught_msg;
  try {
    except_factory.Throw(msg);
  } catch (Exception& e) {
    caught_msg = e.what();
  }
  EXPECT_EQ(msg, caught_msg);
}

TEST(NullExceptionFactory, NoThrow) {
  NullExceptionFactory except_factory;
  auto throw_exception = [&except_factory]() { except_factory.Throw(""); };
  EXPECT_NO_THROW(throw_exception());
}
