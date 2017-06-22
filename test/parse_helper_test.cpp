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

#include "lms1xx/parse_helpers.h"
#include <gtest/gtest.h>
#include <string>
#include <sstream>

class ParseHelperTest : public ::testing::Test
{
  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
  }
};

TEST_F(ParseHelperTest, uint8_t)
{
  const char *str = "00 01 FF 1F";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  uint8_t res;
  nextToken(&mem, res);
  EXPECT_EQ(res, 0);
  nextToken(&mem, res);
  EXPECT_EQ(res, 1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0xff);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0x1f);
}

TEST_F(ParseHelperTest, uint16_t)
{
  const char *str = "0001 FF1F";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  uint16_t res;
  nextToken(&mem, res);
  EXPECT_EQ(res, 1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0xff1f);
}

TEST_F(ParseHelperTest, uint32_t)
{
  const char *str = "0001FF1F 00000001 FFFFFFFF";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  uint32_t res;
  nextToken(&mem, res);
  EXPECT_EQ(res, 0x1ff1f);
  nextToken(&mem, res);
  EXPECT_EQ(res, 1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0xffffffff);
}

TEST_F(ParseHelperTest, int32_t)
{
  const char *str = "-00000001 00000001 FFFFFFFF 00123456";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  int32_t res;
  nextToken(&mem, res);
  EXPECT_EQ(res, -1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0xffffffff);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0x00123456);
}

TEST_F(ParseHelperTest, int16_t)
{
  const char *str = "-0001 0001 FFFF 0456";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  int16_t res;
  nextToken(&mem, res);
  EXPECT_EQ(res, -1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 1);
  nextToken(&mem, res);
  EXPECT_EQ(res, -1);
  nextToken(&mem, res);
  EXPECT_EQ(res, 0x0456);
}

uint32_t toHex(float f)
{
  return *reinterpret_cast<uint32_t *>(&f);
}

TEST_F(ParseHelperTest, float)
{
  std::stringstream ss;
  float test1 = 1;
  float test2 = 1.5;
  float test3 = -1337.1337;
  ss << std::hex << toHex(test1) << " " << std::hex << toHex(test2) << " " << std::hex << toHex(test3);
  std::string str = ss.str();
  char *mem = (char *)malloc(strlen(str.c_str()) + 1);
  memcpy(mem, str.c_str(), strlen(str.c_str()) + 1);
  float res;
  nextToken(&mem, res);
  EXPECT_EQ(res, test1);
  nextToken(&mem, res);
  EXPECT_EQ(res, test2);
  nextToken(&mem, res);
  EXPECT_EQ(res, test3);
}

TEST_F(ParseHelperTest, string)
{
  const char *str = "test1 test2 12ff";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  std::string res;
  nextToken(&mem, res);
  EXPECT_EQ(res, std::string("test1"));
  nextToken(&mem, res);
  EXPECT_EQ(res, std::string("test2"));
  nextToken(&mem, res);
  EXPECT_EQ(res, std::string("12ff"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
