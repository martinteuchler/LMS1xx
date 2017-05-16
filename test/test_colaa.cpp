/*
 * Copyright (c) 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include "lms1xx/colaa_structs.h"

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
  ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::PARSE_ERROR);
}


TEST_F(CoLaATest, single_digit_error_normal)
{
  const char *buf = "3";
  ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
}

TEST_F(CoLaATest, single_digit_error_non_number)
{
  {
    const char *buf = "a";
    ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::PARSE_ERROR);
  }
  {
    const char *buf = ".";
    ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, single_digit_error_double_digit)
{
  const char *buf = "30";
  ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
}
TEST_F(CoLaATest, single_digit_error_mixed)
{
  {
    const char *buf = "3a";
    ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::Sopas_Error_VARIABLE_UNKNONWINDEX);
  }
  {
    const char *buf = "a3";
    ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, double_digit_error_null)
{
  const char *buf = nullptr;
  ASSERT_EQ(CoLaASopasError::parseError(buf, true), CoLaASopasError::PARSE_ERROR);
}

TEST_F(CoLaATest, double_digit_error_normal)
{
  const char *buf = "23";
  ASSERT_EQ(CoLaASopasError::parseError(buf, true), CoLaASopasError::Sopas_Error_HubAddressAddressExceeded);
}

TEST_F(CoLaATest, double_digit_error_normal_2)
{
  const char *buf = "23a";
  ASSERT_EQ(CoLaASopasError::parseError(buf, true), CoLaASopasError::Sopas_Error_HubAddressAddressExceeded);
}
TEST_F(CoLaATest, double_digit_error_non_numeric)
{
  {
    const char *buf = "..";
    ASSERT_EQ(CoLaASopasError::parseError(buf, false), CoLaASopasError::PARSE_ERROR);
  }
  {
    const char *buf = "aa";
    ASSERT_EQ(CoLaASopasError::parseError(buf, true), CoLaASopasError::PARSE_ERROR);
  }
}

TEST_F(CoLaATest, double_digit_error_mixed)
{
  const char *buf = "2a";
  ASSERT_EQ(CoLaASopasError::parseError(buf, true), CoLaASopasError::PARSE_ERROR);
}

int main(int argc, char**argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
