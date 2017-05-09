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
#include "LMS1xx/lms_structs.h"

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

void CoLaA::set_scan_config(const ScanConfig &cfg)
{
  std::string command = SET_SCAN_CFG_COMMAND + " " + build_scan_cfg(cfg);
  send_command(command);

  read_back();
}

void CoLaA::set_scan_data_config(const ScanDataConfig &cfg)
{
  std::string command = SET_SCAN_DATA_CFG_COMMAND + " " + build_scan_data_cfg(cfg);
  send_command(command);

  read_back();
}

ScanConfig CoLaA::get_scan_config()
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

CoLaAStatus::Status CoLaA::query_status()
{
  send_command(QUERY_STATUS_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  CoLaAStatus::Status status = CoLaAStatus::Error;
  if (read_back(buf, len) && len > 10)
  {
    int ret;
    sscanf((buf + 10), "%d", &ret);
    status = static_cast<CoLaAStatus::Status>(ret);
  }
  return status;
}

ScanOutputRange CoLaA::get_scan_output_range()
{
  send_command(READ_SCAN_OUTPUT_RANGE_COMMAND);

  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  read_back(buf, len);
  char *parsable = &buf[0];
  ScanOutputRange range;
  next_token(&parsable); // command type
  next_token(&parsable); // command name
  next_token(&parsable, range.num_sectors);
  next_token(&parsable, range.angular_resolution);
  next_token(&parsable, range.start_angle);
  next_token(&parsable, range.stop_angle);
  return range;
}

void CoLaA::scan_continuous(bool start)
{
  std::string command = START_CONT_COMMAND + " " + std::to_string(static_cast<int>(start));
  send_command(command);
  read_back();
}

bool CoLaA::get_scan_data(void *scan_data)
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

void CoLaA::do_login(std::string user_class, std::string password)
{
   std::string command = LOGIN_COMMAND + " " + user_class + " " + password;
   send_command(command);
}

ScanConfig CoLaA::parse_scan_cfg(char *buf, size_t len)
{
  ScanConfig cfg;
  next_token(&buf); // Command type
  next_token(&buf); // Command name
  next_token(&buf, cfg.scan_frequency);
  next_token(&buf, cfg.num_sectors);
  next_token(&buf, cfg.angualr_resolution);
  next_token(&buf, cfg.start_ange);
  next_token(&buf, cfg.stop_angle);
  return cfg;
}

std::string CoLaA::build_scan_cfg(const ScanConfig &cfg) const
{
  if (cfg.num_sectors > 1)
  {
    logWarn("This method does not support configuring multiple sectors");
  }
  std::stringstream ss;
  ss << std::uppercase << std::hex << cfg.scan_frequency << " +" << std::dec << std::min(cfg.num_sectors, (int16_t)1) << " "
     << std::hex << cfg.angualr_resolution << " " << cfg.start_ange << " " << cfg.stop_angle;
  logDebug("TX: %s", ss.str().c_str());
  return ss.str();
}

std::string CoLaA::build_scan_data_cfg(const ScanDataConfig &cfg) const
{
  std::stringstream ss;
  ss << build_scan_data_cfg_output_channel(cfg.output_channel);
  ss << " " << cfg.remission;
  ss << " " << cfg.resolution;
  ss << " 0"; // Resolution, always 0
  ss << " " << build_scan_data_cfg_encoder(cfg.encoder);
  ss << " " << cfg.position;
  ss << " " << cfg.device_name;
  ss << " " << cfg.comment;
  ss << " " << cfg.timestamp;
  ss << " +" << cfg.output_interval;
  return ss.str();
}

std::string CoLaA::build_scan_data_cfg_output_channel(int ch) const
{
  std::stringstream ss;
  ss << std::setw(2) << std::setfill('0') << ch << " 00";
  return ss.str();
}

std::string CoLaA::build_scan_data_cfg_encoder(int enc) const
{
  if (enc)
  {
    std::stringstream ss;
    ss << std::setw(2) << std::setfill('0') << enc << " 00";
    return ss.str();
  }
  return "00 00"; // Data sheet says "No encoder: 0, but that produces an error"
}

// Dangerous hack!
void copy_vector(std::vector<uint8_t> &src, uint16_t *dest)
{
  for (size_t i = 0; i < src.size(); ++i)
  {
    dest[i] = src[i];
  }
}

void CoLaA::parse_scan_data(char *buffer, void *__data) const
{
  scanData *data = (scanData *)__data;
  ScanDataHeader header = parse_scan_data_header(&buffer);
  (void)header; // Unused
  parse_scan_data_encoderdata(&buffer);
  std::vector<ChannelData<uint16_t> > channels_16bit = ChannelData<uint16_t>::parse_scan_data_channels(&buffer);
  std::vector<ChannelData<uint8_t> > channels_8bit = ChannelData<uint8_t>::parse_scan_data_channels(&buffer);

  // These seem to contain the dist values
  for (size_t i = 0; i < channels_16bit.size(); ++i)
  {
    if (channels_16bit[i].header.contents == "DIST1")
    {
      data->dist_len1 = channels_16bit[i].header.data_count;
      memcpy(data->dist1, channels_16bit[i].data.data(), channels_16bit[i].data.size() * sizeof(uint16_t));
    }
    else if (channels_16bit[i].header.contents == "DIST2")
    {
      data->dist_len2 = channels_16bit[i].header.data_count;
      memcpy(data->dist2, channels_16bit[i].data.data(), channels_16bit[i].data.size() * sizeof(uint16_t));
    }
    else if (channels_16bit[i].header.contents == "RSSI1")
    {
      data->rssi_len1 = channels_16bit[i].header.data_count;
      memcpy(data->rssi1, channels_16bit[i].data.data(), channels_16bit[i].data.size() * sizeof(uint16_t));
    }
    else if (channels_16bit[i].header.contents == "RSSI2")
    {
      data->rssi_len2 = channels_16bit[i].header.data_count;
      memcpy(data->rssi2, channels_16bit[i].data.data(), channels_16bit[i].data.size() * sizeof(uint16_t));
    }
  }


  // These seem to contain the RSSI values
  for (size_t i = 0; i < channels_8bit.size(); ++i)
  {
    if (channels_8bit[i].header.contents == "DIST1")
    {
      data->dist_len1 = channels_8bit[i].header.data_count;
      copy_vector(channels_8bit[i].data, data->dist1);
    }
    else if (channels_8bit[i].header.contents == "DIST2")
    {
      data->dist_len2 = channels_8bit[i].header.data_count;
      copy_vector(channels_8bit[i].data, data->dist2);
    }
    else if (channels_8bit[i].header.contents == "RSSI1")
    {
      data->rssi_len1 = channels_8bit[i].header.data_count;
      copy_vector(channels_8bit[i].data, data->rssi1);
    }
    else if (channels_8bit[i].header.contents == "RSSI2")
    {
      data->rssi_len2 = channels_8bit[i].header.data_count;
      copy_vector(channels_8bit[i].data, data->rssi2);
    }
  }
}

ScanDataHeader CoLaA::parse_scan_data_header(char **buf) const
{
  ScanDataHeader header;
  next_token(buf); // Command Type, either sRN or sNA
  next_token(buf); // Command: LMDscandata

  next_token(buf, header.version_number);

  next_token(buf, header.device.device_number);
  next_token(buf, header.device.serial_number);
  next_token(buf, header.device.device_status_1);
  next_token(buf, header.device.device_status_2);

  next_token(buf, header.status_info.telegram_counter);
  next_token(buf, header.status_info.scan_counter);
  next_token(buf, header.status_info.time_since_startup);
  next_token(buf, header.status_info.time_of_transmission);
  next_token(buf, header.status_info.status_digitalin_1);
  next_token(buf, header.status_info.status_digitalin_2);
  next_token(buf, header.status_info.status_digitalout_1);
  next_token(buf, header.status_info.status_digitalout_2);
  next_token(buf, header.status_info.reserved);

  next_token(buf, header.frequencies.scan_frequency);
  next_token(buf, header.frequencies.measurement_frequency);
  // Extracted 18 fields
  return header;
}

void CoLaA::parse_scan_data_encoderdata(char **buf) const
{
   uint16_t num_encoders = 0;
   next_token(buf, num_encoders);
   logDebug("Got %ud encoders", num_encoders);
   for (uint16_t i = 0; i < num_encoders; ++ i)
   {
     uint32_t encoder_position = 0;
     uint16_t encoder_speed = 0;
     next_token(buf, encoder_position);
     next_token(buf, encoder_speed);
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
  ssize_t len = read(socket_fd_, buf, buflen);
  bool success = buf[0] == STX;
  if ((len == 7 || len == 8) && strncmp(&buf[1], "sFA ", 4) == 0)
  {
    // This is an error message
    CoLaASopasError::SopasError err = CoLaASopasError::parse_error(&buf[5], len == 8);
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

bool CoLaA::read_back()
{
  char buf[DEF_BUF_LEN];
  size_t len = (sizeof buf);
  return read_back(buf, len);
}
