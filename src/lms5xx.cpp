#include "LMS1xx/lms5xx.h"
#include <sstream>
#include <iomanip>

LMS5xx::LMS5xx()
{
  SET_ECHO_FILTER_COMMAND = "sWN FREchoFilter";
}

void LMS5xx::setEchoFilter(LMS5xx::EchoFilter filter)
{
  std::stringstream cmd;
  cmd << SET_ECHO_FILTER_COMMAND << " " << filter;
  sendCommand(cmd.str());
  readBack();
}

std::string LMS5xx::buildScanDataCfgOutputChannel(int ch) const
{
  // Set via Echo Filter
  // Set this value to zero for lms5xx
  // Spec sheet says 0 but that doesn't work...
  //return "0";
  return CoLaA::buildScanDataCfgOutputChannel(ch);
}
