#include "LMS1xx/colaa.h"

#include <sys/socket.h>
#include <netinet/in.h> // sockaddr
#include <arpa/inet.h> // inet_pton
#include <iostream>

#include "LMS1xx/lms_buffer.h"

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

bool CoLaA::is_connected() const
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

    do_login(LOGIN_USER_AUTHORIZED, LOGIN_PASS_AUTHORIZED);

    FD_ZERO(&readset);
    FD_SET(socket_fd_, &readset);
    result = select(socket_fd_ + 1, &readset, NULL, NULL, &timeout);

  }
  while (result <= 0);

  read_back();
}

void CoLaA::start_device()
{
  send_command(START_DEVICE_COMMAND);
  read_back();
}

void CoLaA::start_measurement()
{
  send_command(START_MEASUREMENT_COMMAND);
  read_back();
}

void CoLaA::stop_measurement()
{
  send_command(STOP_MEASUREMENT_COMMAND);
  read_back();
}

void CoLaA::set_scan_config(const scanCfg &cfg)
{
  std::string command = SET_SCAN_CFG_COMMAND + " " + build_scan_cfg(cfg);
  send_command(command);

  read_back();
}

void CoLaA::set_scan_data_config(const scanDataCfg &cfg)
{
  std::string command = SET_SCAN_DATA_CFG_COMMAND + " " + build_scan_data_cfg(cfg);
  send_command(command);

  read_back();
}

scanCfg CoLaA::get_scan_config()
{
  send_command(READ_SCAN_CFG_COMMAND);
  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  read_back(buf, len);

  return parse_scan_cfg(buf, len);
}

void CoLaA::save_config()
{
  send_command(SAVE_CONFIG_COMMAND);
  read_back();
}

CoLaA::Status CoLaA::query_status()
{
  send_command(QUERY_STATUS_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  CoLaA::Status status = Status::Error;
  if (read_back(buf, len))
  {
    int ret;
    sscanf((buf + 10), "%d", &ret);
    status = static_cast<CoLaA::Status>(ret);
  }
  return status;
}

scanOutputRange CoLaA::get_scan_output_range()
{
  send_command(READ_SCAN_OUTPUT_RANGE_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  read_back(buf, len);
  // TODO: Clean up
  scanOutputRange outputRange;
  sscanf(buf + 1, "%*s %*s %*d %X %X %X", &outputRange.angleResolution,
         &outputRange.startAngle, &outputRange.stopAngle);
  return outputRange;
}

void CoLaA::scan_continuous(bool start)
{
  std::string command = START_CONT_COMMAND + " " + std::to_string(static_cast<int>(start));
  send_command(command);
  read_back();
}

bool CoLaA::get_scan_data(scanData *scan_data)
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
        parse_scan_data(buffer_data, scan_data);
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

void CoLaA::printTest()
{
  std::cout << LOGIN_COMMAND << std::endl;
}

void CoLaA::do_login(std::string user_class, std::string password)
{
   std::string command = LOGIN_COMMAND + " " + user_class + " " + password;
   send_command(command);
}

scanCfg CoLaA::parse_scan_cfg(const char *buf, size_t len)
{
  scanCfg cfg;
  // TODO make more better?
  sscanf(buf + 1, "%*s %*s %X %*d %X %X %X", &cfg.scaningFrequency,
         &cfg.angleResolution, &cfg.startAngle, &cfg.stopAngle);
  return cfg;
}

std::string CoLaA::build_scan_cfg(const scanCfg &cfg) const
{
  // TODO: clean up
  char buf[128];
  sprintf(buf, "%X +1 %X %X %X%c",
          cfg.scaningFrequency, cfg.angleResolution, cfg.startAngle,
          cfg.stopAngle, 0x03);
  logDebug("TX: %s", buf);
  return std::string(buf);
}

std::string CoLaA::build_scan_data_cfg(const scanDataCfg &cfg) const
{
  // TODO: clean up
  char buf[128];
  sprintf(buf, "%02X 00 %d %d 0 %02X 00 %d %d 0 %d +%d%c",
          cfg.outputChannel, cfg.remission ? 1 : 0,
          cfg.resolution, cfg.encoder, cfg.position ? 1 : 0,
          cfg.deviceName ? 1 : 0, cfg.timestamp ? 1 : 0, cfg.outputInterval, 0x03);
  return std::string(buf);
}

void CoLaA::parse_scan_data(char *buffer, scanData *data) const
{
  char* tok = strtok(buffer, " "); //Type of command
  tok = strtok(NULL, " "); //Command
  tok = strtok(NULL, " "); //VersionNumber
  tok = strtok(NULL, " "); //DeviceNumber
  tok = strtok(NULL, " "); //Serial number
  tok = strtok(NULL, " "); //DeviceStatus
  tok = strtok(NULL, " "); //MessageCounter
  tok = strtok(NULL, " "); //ScanCounter
  tok = strtok(NULL, " "); //PowerUpDuration
  tok = strtok(NULL, " "); //TransmissionDuration
  tok = strtok(NULL, " "); //InputStatus
  tok = strtok(NULL, " "); //OutputStatus
  tok = strtok(NULL, " "); //ReservedByteA
  tok = strtok(NULL, " "); //ScanningFrequency
  tok = strtok(NULL, " "); //MeasurementFrequency
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " ");
  tok = strtok(NULL, " "); //NumberEncoders
  int NumberEncoders;
  sscanf(tok, "%d", &NumberEncoders);
  for (int i = 0; i < NumberEncoders; i++)
  {
    tok = strtok(NULL, " "); //EncoderPosition
    tok = strtok(NULL, " "); //EncoderSpeed
  }

  tok = strtok(NULL, " "); //NumberChannels16Bit
  int NumberChannels16Bit;
  sscanf(tok, "%d", &NumberChannels16Bit);
  logDebug("NumberChannels16Bit : %d", NumberChannels16Bit);

  for (int i = 0; i < NumberChannels16Bit; i++)
  {
    int type = -1; // 0 DIST1 1 DIST2 2 RSSI1 3 RSSI2
    char content[6];
    tok = strtok(NULL, " "); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = 2;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 3;
    }
    tok = strtok(NULL, " "); //ScalingFactor
    tok = strtok(NULL, " "); //ScalingOffset
    tok = strtok(NULL, " "); //Starting angle
    tok = strtok(NULL, " "); //Angular step width
    tok = strtok(NULL, " "); //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);
    logDebug("NumberData : %d", NumberData);

    if (type == 0)
    {
      data->dist_len1 = NumberData;
    }
    else if (type == 1)
    {
      data->dist_len2 = NumberData;
    }
    else if (type == 2)
    {
      data->rssi_len1 = NumberData;
    }
    else if (type == 3)
    {
      data->rssi_len2 = NumberData;
    }

    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);

      if (type == 0)
      {
        data->dist1[i] = dat;
      }
      else if (type == 1)
      {
        data->dist2[i] = dat;
      }
      else if (type == 2)
      {
        data->rssi1[i] = dat;
      }
      else if (type == 3)
      {
        data->rssi2[i] = dat;
      }

    }
  }

  tok = strtok(NULL, " "); //NumberChannels8Bit
  int NumberChannels8Bit;
  sscanf(tok, "%d", &NumberChannels8Bit);
  logDebug("NumberChannels8Bit : %d\n", NumberChannels8Bit);

  for (int i = 0; i < NumberChannels8Bit; i++)
  {
    int type = -1;
    char content[6];
    tok = strtok(NULL, " "); //MeasuredDataContent
    sscanf(tok, "%s", content);
    if (!strcmp(content, "DIST1"))
    {
      type = 0;
    }
    else if (!strcmp(content, "DIST2"))
    {
      type = 1;
    }
    else if (!strcmp(content, "RSSI1"))
    {
      type = 2;
    }
    else if (!strcmp(content, "RSSI2"))
    {
      type = 3;
    }
    tok = strtok(NULL, " "); //ScalingFactor
    tok = strtok(NULL, " "); //ScalingOffset
    tok = strtok(NULL, " "); //Starting angle
    tok = strtok(NULL, " "); //Angular step width
    tok = strtok(NULL, " "); //NumberData
    int NumberData;
    sscanf(tok, "%X", &NumberData);
    logDebug("NumberData : %d\n", NumberData);

    if (type == 0)
    {
      data->dist_len1 = NumberData;
    }
    else if (type == 1)
    {
      data->dist_len2 = NumberData;
    }
    else if (type == 2)
    {
      data->rssi_len1 = NumberData;
    }
    else if (type == 3)
    {
      data->rssi_len2 = NumberData;
    }
    for (int i = 0; i < NumberData; i++)
    {
      int dat;
      tok = strtok(NULL, " "); //data
      sscanf(tok, "%X", &dat);

      if (type == 0)
      {
        data->dist1[i] = dat;
      }
      else if (type == 1)
      {
        data->dist2[i] = dat;
      }
      else if (type == 2)
      {
        data->rssi1[i] = dat;
      }
      else if (type == 3)
      {
        data->rssi2[i] = dat;
      }
    }
  }
}

void CoLaA::send_command(const std::string &command) const
{
  send_command(command.c_str());
}

void CoLaA::send_command(const char *command) const
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

bool CoLaA::read_back(char *buf, size_t &buflen)
{
  if (!buf) {
    logDebug("No buffer supplied");
    return false;
  }
  int len = read(socket_fd_, buf, buflen);
  bool success = buf[0] == STX;
  if (!success)
    logWarn("invalid packet recieved");
  buf[len > 0 ? len : 0] = 0;
  logDebug("RX: %s", buf);
  if (len >= 0)
    buflen = static_cast<size_t>(len);
  return success;
}

bool CoLaA::read_back()
{
  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  return read_back(buf, len);
}
