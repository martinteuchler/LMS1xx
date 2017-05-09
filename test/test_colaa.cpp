#include <gtest/gtest.h>
#include "LMS1xx/colaa.h"

class CoLaATest : public ::testing::Test
{
  virtual void SetUp()
  {

  }

  virtual void TearDown()
  {

  }
};

TEST_F(CoLaATest, single_digit_error_null)
{
  const char *buf = nullptr;
  ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::PARSE_ERROR);
}


TEST_F(CoLaATest, single_digit_error_normal)
{
  const char *buf = "3";
  ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
}

TEST_F(CoLaATest, single_digit_error_non_number)
{
  {
    const char *buf = "a";
    ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::PARSE_ERROR);
  }
  {
    const char *buf = ".";
    ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, single_digit_error_double_digit)
{
  const char *buf = "30";
  ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
}
TEST_F(CoLaATest, single_digit_error_mixed)
{
  {
    const char *buf = "3a";
    ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
  }
  {
    const char *buf = "a3";
    ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, double_digit_error_null)
{
  const char *buf = nullptr;
  ASSERT_EQ(CoLaA::parse_error(buf, true), CoLaA::SopasError::PARSE_ERROR);
}

TEST_F(CoLaATest, double_digit_error_normal)
{
  const char *buf = "23";
  ASSERT_EQ(CoLaA::parse_error(buf, true), CoLaA::SopasError::Sopas_Error_HubAddressAddressExceeded);
}

TEST_F(CoLaATest, double_digit_error_normal_2)
{
  const char *buf = "23a";
  ASSERT_EQ(CoLaA::parse_error(buf, true), CoLaA::SopasError::Sopas_Error_HubAddressAddressExceeded);
}
TEST_F(CoLaATest, double_digit_error_non_numeric)
{
  {
    const char *buf = "..";
    ASSERT_EQ(CoLaA::parse_error(buf, false), CoLaA::SopasError::PARSE_ERROR);
  }
  {
    const char *buf = "aa";
    ASSERT_EQ(CoLaA::parse_error(buf, true), CoLaA::SopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, double_digit_error_mixed)
{
  const char *buf = "2a";
  ASSERT_EQ(CoLaA::parse_error(buf, true), CoLaA::SopasError::PARSE_ERROR);
}

int main(int argc, char**argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
