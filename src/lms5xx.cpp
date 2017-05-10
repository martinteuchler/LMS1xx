#include "../include/LMS1xx/lms5xx.h"
#include <sstream>
#include <iomanip>

LMS5xx::LMS5xx()
{
  SET_ECHO_FILTER_COMMAND = "sWN FREchoFilter";
}

void LMS5xx::set_echo_filter(LMS5xx::EchoFilter filter)
{
  std::stringstream cmd;
  cmd << SET_ECHO_FILTER_COMMAND << " " << filter;
  send_command(cmd.str());
  read_back();
}

std::string LMS5xx::build_scan_data_cfg_output_channel(int ch) const
{
  // Set via Echo Filter
  // Set this value to zero for lms5xx
  // Spec sheet says 0 but that doesn't work...
  //return "0";
  return CoLaA::build_scan_data_cfg_output_channel(ch);
}
