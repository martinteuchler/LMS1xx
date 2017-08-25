#include <lms1xx/colaa.h>
#include <gtest/gtest.h>
#include <fstream>
#include <sstream>

class MRS1000ScanDataTest : public ::testing::Test
{
public:
  bool parseScanData(char *buffer , void *__data)
  {
    return colaa.parseScanData(buffer, __data);
  }

  CoLaA colaa;
};

TEST_F(MRS1000ScanDataTest, three_echo_buffer)
{
  std::ifstream reader("test/mrs1000.txt", std::ios::binary);
  std::stringstream ss;
  ss << reader.rdbuf();
  std::string buf_str = ss.str();
  char *mem = (char *)malloc(strlen(buf_str.c_str()) + 1);
  memcpy(mem, buf_str.c_str(), strlen(buf_str.c_str()) + 1);
  ScanData data;
  this->parseScanData(mem, &data);
  EXPECT_EQ(data.header.status_info.layer_angle, CoLaALayers::Layer4);
  EXPECT_EQ(data.header.frequencies.scan_frequency, 5000);
  EXPECT_EQ(data.header.device.serial_number, 17300004);
  EXPECT_EQ(data.ch16bit.size(), 3);
  EXPECT_EQ(data.ch8bit.size(), 3);
  EXPECT_EQ(data.ch16bit[0].header.contents, "DIST1");
  EXPECT_EQ(data.ch8bit[0].header.contents, "RSSI1");
  EXPECT_EQ(data.ch16bit[0].header.data_count, 1101);
  EXPECT_EQ(data.ch16bit[0].data.size(), 1101);
  EXPECT_EQ(data.ch8bit[0].header.data_count, 1101);
  EXPECT_EQ(data.ch8bit[0].data.size(), 1101);
  EXPECT_EQ(data.ch16bit[1].data[47], 2566);
  EXPECT_EQ(data.ch8bit[0].data[100], 232);
}

TEST_F(MRS1000ScanDataTest, enable_reply_parsing)
{
  const char *str = "\x02sEA LMDscandata 1\x03";
  char *mem = (char *)malloc(strlen(str) + 1);
  memcpy(mem, str, strlen(str) + 1);
  EXPECT_FALSE(this->parseScanData(mem, NULL));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
