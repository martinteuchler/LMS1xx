#include "LMS1xx/colaa.h"

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h> // sockaddr
#include <arpa/inet.h> // inet_pton
#include <console_bridge/console.h>
#include <sstream>
#include <iomanip>
#include <inttypes.h>

#include "LMS1xx/lms_buffer.h"
#include "LMS1xx/parse_helpers.h"

constexpr uint8_t STX = 0x02; //Start transmission marker
constexpr uint8_t ETX = 0x03; //End transmission marker
constexpr size_t DEF_BUF_LEN = 128; // Default buffer size

CoLaA::CoLaA() : connected_(false)
{
  buffer_ = new LMSBuffer();
  LOGIN_COMMAND = "sMN SetAccessMode";
  LOGIN_USER_MAINT = "02"; // Maintenance
  LOGIN_USER_AUTHORIZED = "03"; // Authorized Client
  LOGIN_USER_SERVICE = "04"; // Service
  LOGIN_PASS_MAINT = "B21ACE26"; // Maintenance
  LOGIN_PASS_AUTHORIZED = "F4724744"; // Authorized Client
  LOGIN_PASS_SERVICE = "81BE23AA"; // Service

  READ_SCAN_CFG_COMMAND = "sRN LMPscancfg";
  SET_SCAN_CFG_COMMAND = "sMN mLMPsetscancfg";
  SET_SCAN_DATA_CFG_COMMAND = "sWN LMDscandatacfg";
  SAVE_CONFIG_COMMAND = "sMN mEEwriteall";

  START_MEASUREMENT_COMMAND = "sMN LMCstartmeas";
  STOP_MEASUREMENT_COMMAND = "sMN LMCstopmeas";
  START_CONT_COMMAND = "sEN LMDscandata";

  QUERY_STATUS_COMMAND = "sRN STlms";

  READ_SCAN_OUTPUT_RANGE_COMMAND = "sRN LMPoutputRange";

  START_DEVICE_COMMAND = "sMN Run";
}

CoLaA::~CoLaA()
{
  delete buffer_;
}

void CoLaA::connect(std::string host, int port)
{
  if (!connected_)
  {
    logDebug("Creating non-blocking socket.");
    socket_fd_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_fd_)
    {
      struct sockaddr_in stSockAddr;
      stSockAddr.sin_family = PF_INET;
      stSockAddr.sin_port = htons(port);
      inet_pton(AF_INET, host.c_str(), &stSockAddr.sin_addr);

      logDebug("Connecting socket to laser.");
      int ret = ::connect(socket_fd_, (struct sockaddr *) &stSockAddr, sizeof(stSockAddr));

      if (ret == 0)
      {
        connected_ = true;
        logDebug("Connected succeeded.");
      }
    }
  }
}

void CoLaA::disconnect()
{
  if (connected_)
  {
    close(socket_fd_);
    connected_ = false;
  }
}

bool CoLaA::isConnected() const
{
  return connected_;
}

void CoLaA::login()
{
  fd_set readset;
  struct timeval timeout;
  ssize_t result = -1;

  do   //loop until data is available to read
  {
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    doLogin(LOGIN_USER_AUTHORIZED, LOGIN_PASS_AUTHORIZED);

    FD_ZERO(&readset);
    FD_SET(socket_fd_, &readset);
    result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

  }
  while (result <= 0);

  readBack();
}

void CoLaA::startDevice()
{
  sendCommand(START_DEVICE_COMMAND);
  readBack();
}

void CoLaA::startMeasurement()
{
  sendCommand(START_MEASUREMENT_COMMAND);
  readBack();
}

void CoLaA::stopMeasurement()
{
  sendCommand(STOP_MEASUREMENT_COMMAND);
  readBack();
}

void CoLaA::setScanConfig(const ScanConfig &cfg)
{
  std::string command = SET_SCAN_CFG_COMMAND + " " + buildScanCfg(cfg);
  sendCommand(command);

  readBack();
}

void CoLaA::setScanDataConfig(const ScanDataConfig &cfg)
{
  std::string command = SET_SCAN_DATA_CFG_COMMAND + " " + buildScanDataCfg(cfg);
  sendCommand(command);

  readBack();
}

ScanConfig CoLaA::getScanConfig()
{
  sendCommand(READ_SCAN_CFG_COMMAND);
  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  readBack(buf, len);

  return parseScanCfg(buf, len);
}

void CoLaA::saveConfig()
{
  sendCommand(SAVE_CONFIG_COMMAND);
  readBack();
}

CoLaAStatus::Status CoLaA::queryStatus()
{
  sendCommand(QUERY_STATUS_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  CoLaAStatus::Status status = CoLaAStatus::Error;
  if (readBack(buf, len) && len > 10)
  {
    int ret;
    sscanf((buf + 10), "%d", &ret);
    status = static_cast<CoLaAStatus::Status>(ret);
  }
  return status;
}

ScanOutputRange CoLaA::getScanOutputRange()
{
  sendCommand(READ_SCAN_OUTPUT_RANGE_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  readBack(buf, len);
  char *parsable = &buf[0];
  ScanOutputRange range;
  nextToken(&parsable); // command type
  nextToken(&parsable); // command name
  nextToken(&parsable, range.num_sectors);
  nextToken(&parsable, range.angular_resolution);
  nextToken(&parsable, range.start_angle);
  nextToken(&parsable, range.stop_angle);
  return range;
}

void CoLaA::scanContinuous(bool start)
{
  std::string command = START_CONT_COMMAND + " " + std::to_string(static_cast<int>(start));
  sendCommand(command);
  readBack();
}

bool CoLaA::getScanData(void *scan_data)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(socket_fd_, &rfds);

  // Block a total of up to 100ms waiting for more data from the laser.
  while (1)
  {
    // Would be great to depend on linux's behaviour of updating the timeval, but unfortunately
    // that's non-POSIX (doesn't work on OS X, for example).
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    logDebug("entering select()", tv.tv_usec);
    int retval = select(socket_fd_ + 1, &rfds, NULL, NULL, &tv);
    logDebug("returned %d from select()", retval);
    if (retval)
    {
      buffer_->readFrom(socket_fd_);

      // Will return pointer if a complete message exists in the buffer,
      // otherwise will return null.
      char* buffer_data = buffer_->getNextBuffer();

      if (buffer_data)
      {
        parseScanData(buffer_data, scan_data);
        buffer_->popLastBuffer();
        return true;
      }
    }
    else
    {
      // Select timed out or there was an fd error.
      return false;
    }
  }
}

void CoLaA::doLogin(std::string user_class, std::string password)
{
   std::string command = LOGIN_COMMAND + " " + user_class + " " + password;
   sendCommand(command);
}

ScanConfig CoLaA::parseScanCfg(char *buf, size_t len)
{
  ScanConfig cfg;
  nextToken(&buf); // Command type
  nextToken(&buf); // Command name
  nextToken(&buf, cfg.scan_frequency);
  nextToken(&buf, cfg.num_sectors);
  nextToken(&buf, cfg.angualar_resolution);
  nextToken(&buf, cfg.start_angle);
  nextToken(&buf, cfg.stop_angle);
  return cfg;
}

std::string CoLaA::buildScanCfg(const ScanConfig &cfg) const
{
  if (cfg.num_sectors > 1)
  {
    logWarn("This method does not support configuring multiple sectors");
  }
  std::stringstream ss;
  ss << std::uppercase << std::hex << cfg.scan_frequency << " +" << std::dec << std::min(cfg.num_sectors, (int16_t)1) << " "
     << std::hex << cfg.angualar_resolution << " " << cfg.start_angle << " " << cfg.stop_angle;
  logDebug("TX: %s", ss.str().c_str());
  return ss.str();
}

std::string CoLaA::buildScanDataCfg(const ScanDataConfig &cfg) const
{
  std::stringstream ss;
  ss << buildScanDataCfgOutputChannel(cfg.output_channel);
  ss << " " << cfg.remission;
  ss << " " << cfg.resolution;
  ss << " 0"; // Resolution, always 0
  ss << " " << buildScanDataCfgEncoder(cfg.encoder);
  ss << " " << cfg.position;
  ss << " " << cfg.device_name;
  ss << " " << cfg.comment;
  ss << " " << cfg.timestamp;
  ss << " +" << cfg.output_interval;
  return ss.str();
}

std::string CoLaA::buildScanDataCfgOutputChannel(int ch) const
{
  std::stringstream ss;
  ss << std::setw(2) << std::setfill('0') << ch << " 00";
  return ss.str();
}

std::string CoLaA::buildScanDataCfgEncoder(int enc) const
{
  if (enc)
  {
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << enc << " 00";
    return ss.str();
  }
  return "00 00"; // Data sheet says "No encoder: 0, but that produces an error"
}

void CoLaA::parseScanData(char *buffer, void *__data) const
{
  ScanData *data = (ScanData *)__data;
  data->header = parseScanDataHeader(&buffer);
  parseScanDataEncoderdata(&buffer);
  data->ch16bit = ChannelData<uint16_t>::parseScanDataChannels(&buffer);
  data->ch8bit = ChannelData<uint8_t>::parseScanDataChannels(&buffer);
}

ScanDataHeader CoLaA::parseScanDataHeader(char **buf) const
{
  ScanDataHeader header;
  nextToken(buf); // Command Type, either sRN or sNA
  nextToken(buf); // Command: LMDscandata

  nextToken(buf, header.version_number);

  nextToken(buf, header.device.device_number);
  nextToken(buf, header.device.serial_number);
  nextToken(buf, header.device.device_status_1);
  nextToken(buf, header.device.device_status_2);

  nextToken(buf, header.status_info.telegram_counter);
  nextToken(buf, header.status_info.scan_counter);
  nextToken(buf, header.status_info.time_since_startup);
  nextToken(buf, header.status_info.time_of_transmission);
  nextToken(buf, header.status_info.status_digitalin_1);
  nextToken(buf, header.status_info.status_digitalin_2);
  nextToken(buf, header.status_info.status_digitalout_1);
  nextToken(buf, header.status_info.status_digitalout_2);
  nextToken(buf, header.status_info.reserved);

  nextToken(buf, header.frequencies.scan_frequency);
  nextToken(buf, header.frequencies.measurement_frequency);
  // Extracted 18 fields
  return header;
}

void CoLaA::parseScanDataEncoderdata(char **buf) const
{
   uint16_t num_encoders = 0;
   nextToken(buf, num_encoders);
   logDebug("Got %ud encoders", num_encoders);
   for (uint16_t i = 0; i < num_encoders; ++ i)
   {
     uint32_t encoder_position = 0;
     uint16_t encoder_speed = 0;
     nextToken(buf, encoder_position);
     nextToken(buf, encoder_speed);
   }
}

void CoLaA::sendCommand(const std::string &command) const
{
  sendCommand(command.c_str());
}

void CoLaA::sendCommand(const char *command) const
{
  ssize_t written = write(socket_fd_, &STX, 1);
  if (written != 1)
    logWarn("Error");
  written = write(socket_fd_, command, strlen(command));
  if (written < 0 || static_cast<size_t>(written) != strlen(command))
    logWarn("Error");
  written = write(socket_fd_, &ETX, 1);
  if (written != 1)
    logWarn("Error");
}

bool CoLaA::readBack(char *buf, size_t &buflen)
{
  if (!buf) {
    logDebug("No buffer supplied");
    return false;
  }
  ssize_t len = read(socket_fd_, buf, buflen);
  bool success = buf[0] == STX;
  if ((len == 7 || len == 8) && strncmp(&buf[1], "sFA ", 4) == 0)
  {
    // This is an error message
    CoLaASopasError::SopasError err = CoLaASopasError::parseError(&buf[5], len == 8);
    logWarn("Received error code %d", err);
  }
  if (!success)
    logWarn("invalid packet recieved");
  buf[len > 0 ? len : 0] = 0;
  logDebug("RX: %s", buf);
  if (len >= 0)
    buflen = static_cast<size_t>(len);
  else
    buflen = 0;
  return success;
}

bool CoLaA::readBack()
{
  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  return readBack(buf, len);
}
