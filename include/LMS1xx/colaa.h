#ifndef COLAA_H
#define COLAA_H

#include <string>
#include <stdint.h>
#include "LMS1xx/lms_structs.h"
#include <vector>

class LMSBuffer;

class CoLaA
{
public:
  enum Status : uint8_t
  {
    Undefined = 0,
    Initialisation = 1,
    Configuration = 2,
    Idle = 3,
    Rotated = 4,
    InPreparation = 5,
    Ready = 6,
    ReadyForMeasurement = 7,
    Error = 8 // Not from spec, failed to read status at all
  };

  enum SopasError
  {
    Sopas_Ok = 0,
    Sopas_Error_METHODIN_ACCESDENIED = 1,
    Sopas_Error_METHODIN_UNKNOWNINDEX = 2,
    Sopas_Error_VARIABLE_UNKNONWINDEX = 3,
    Sopas_Error_LOCALCONDITIONFAILED = 4,
    Sopas_Error_INVALID_DATA = 5,
    Sopas_Error_UNKNOWN_Error = 6,
    Sopas_Error_BUFFER_OVERFLOW = 7,
    Sopas_Error_BUFFER_UNDERFLOW = 8,
    Sopas_Error_ERROR_UNKNOWN_TYPE = 9,
    Sopas_Error_VARIABLE_WRITE_aCCESSDENIED = 10,
    Sopas_Error_UNKNOWN_CMD_FOR_NAMESERVER = 11,
    Sopas_Error_UNKNOWN_COLA_COMMAND = 12,
    Sopas_Error_METHODIN_SERVER_BUSY = 13,
    Sopas_Error_FLEX_OUT_OF_BOUNDS = 14,
    Sopas_Error_EVENTREG_UNKNOWNINDEX = 15,
    Sopas_Error_COLA_A_VALUE_OVERFLOW = 16,
    Sopas_Error_COLA_A_INVALID_CHARACTER = 17,
    Sopas_Error_OSAI_NO_MESSAGE = 18,
    Sopas_Error_OSAI_NO_ANSWER_MESSAGE = 19,
    Sopas_Error_INTERNAL = 20,
    Sopas_Error_HubAddressCorrupted = 21,
    Sopas_Error_HubAddressDecoding = 22,
    Sopas_Error_HubAddressAddressExceeded = 23,
    Sopas_Error_HubAddressBlankExpected = 24,
    Sopas_Error_AsyncMethodsAreSuppressed = 25,
    Sopas_Error_ComplexArraysNotSupported = 26,
    PARSE_ERROR = 999 // Failed to parse error code
  };

  struct ScanDataHeader
  {
    uint16_t version_number;

    // Device section
    struct
    {
      uint16_t device_number;
      uint32_t serial_number;
      uint8_t device_status_1;
      uint8_t device_status_2;
    } device;

    // Status info section
    struct
    {
      uint16_t telegram_counter;
      uint16_t scan_counter;
      uint32_t time_since_startup;
      uint32_t time_of_transmission;
      uint8_t status_digitalin_1; // LMS1xx, LMS5xx, TiM5xx only
      uint8_t status_digitalin_2; // LMS1xx, LMS5xx, TiM5xx only

      // 00 00 all outputs low
      // Rest is device dependant
      uint8_t status_digitalout_1;
      uint8_t status_digitalout_2;

      uint16_t reserved;
    } status_info;

    struct
    {
      uint32_t scan_frequency;
      uint32_t measurement_frequency;
    } frequencies;
  };

  struct ChannelDataHeader
  {
    std::string contents;
    float scale_factor;
    float scale_factor_offset;
    int32_t start_angle; // for 16 bit channels this is uint32_t in the spec sheet, but has
    // negative values in the range regardless
    uint16_t step_size;
    uint16_t data_count;
  };

  template <typename T>
  class ChannelData
  {
  public:
    ChannelDataHeader header;
    std::vector<T> data;
    static std::vector<CoLaA::ChannelData<T> > parse_scan_data_channels(char **buf);
  };

  CoLaA();
  ~CoLaA();

  /*!
  * @brief Connect to a device supporting the CoLaA protocol.
  * @param host device host name or ip address.
  * @param port device port number.
  */
  void connect(std::string host, int port);

  /*!
  * @brief Disconnect from CoLaA device.
  */
  void disconnect();

  /*!
  * @brief Get status of connection.
  * @returns connected or not.
  */
  bool is_connected() const;

  /*!
  * @brief Log into device
  * Increase privilege level, giving ability to change device configuration.
  */
  void login();

  /*!
  * @brief The device is returned to the measurement mode after configuration.
  *
  */
  void start_device();

  /*!
  * @brief Start measurements.
  * After receiving this command the unit starts spinning laser and measuring.
  */
  void start_measurement();

  /*!
  * @brief Stop measurements.
  * After receiving this command LMS1xx unit stop spinning laser and measuring.
  */
  void stop_measurement();

  /*!
  * @brief Set scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @param cfg structure containing scan configuration.
  */
  void set_scan_config(const scanCfg &cfg);

  /*!
  * @brief Set scan data configuration.
  * Set format of scan message returned by device.
  * @param cfg structure containing scan data configuration.
  */
  void set_scan_data_config(const scanDataCfg &cfg);

  /*!
  * @brief Get current scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanCfg structure.
  */
  scanCfg get_scan_config();

  /*!
  * @brief Save data permanently.
  * Parameters are saved in the EEPROM of the LMS and will also be available after the device is switched off and on again.
  *
  */
  void save_config();

  /*!
  * @brief Get current status of device.
  * This should be the same for all sensor types
  * @returns status of device.
  */
  Status query_status();

  /*!
  * @brief Get current output range configuration.
  * Get output range configuration :
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * Should be the same for all sensor types
  * @returns scanOutputRange structure.
  */
  scanOutputRange get_scan_output_range();

  /*!
  * @brief Start or stop continuous data acquisition.
  * After reception of this command device start or stop continuous data stream containing scan messages.
  * @param start true : start false : stop
  */
  void scan_continuous(bool start);

  /*!
  * @brief Receive single scan message.
  * @return true if scan was read successfully, false if error or timeout. False implies that higher level
  *         logic should take correct action such as reopening the connection.
  */
  // TODO: abstract scan_data
  bool get_scan_data(void *scan_data);

  /**
   * @brief parse_error Parse error code from message
   * @param buf expected to be at the first character of the ASCII error code in the message.
   * @param twodigits Whether or not the error code has 2 digits. Can be determined from the message length.
   * If the message has two digits, it is assumed that buf has a size of at least 2.
   * @return The parsed error code or PARSE_ERROR if no error code could be parsed
   */
  static SopasError parse_error(const char *buf, bool twodigits);

protected:
  std::string LOGIN_COMMAND;
  std::string LOGIN_USER_MAINT;
  std::string LOGIN_USER_AUTHORIZED;
  std::string LOGIN_USER_SERVICE;
  std::string LOGIN_PASS_MAINT;
  std::string LOGIN_PASS_AUTHORIZED;
  std::string LOGIN_PASS_SERVICE;

  std::string READ_SCAN_CFG_COMMAND;
  std::string SET_SCAN_CFG_COMMAND;
  std::string SET_SCAN_DATA_CFG_COMMAND;
  std::string SAVE_CONFIG_COMMAND;

  std::string START_MEASUREMENT_COMMAND;
  std::string STOP_MEASUREMENT_COMMAND;
  std::string START_CONT_COMMAND;

  std::string QUERY_STATUS_COMMAND;

  std::string READ_SCAN_OUTPUT_RANGE_COMMAND;

  std::string START_DEVICE_COMMAND;

  void do_login(std::string user_class, std::string password);

  virtual scanCfg parse_scan_cfg(const char *buf, size_t len);
  virtual std::string build_scan_cfg(const scanCfg &cfg) const;
  virtual std::string build_scan_data_cfg(const scanDataCfg &cfg) const;
  virtual std::string build_scan_data_cfg_output_channel(int ch) const;
  virtual std::string build_scan_data_cfg_encoder(int enc) const;

  virtual void parse_scan_data(char *buffer, void *data) const;
  virtual ScanDataHeader parse_scan_data_header(char **buf) const;
  virtual void parse_scan_data_encoderdata(char **buf) const;
  static ChannelDataHeader parse_scan_data_channel_header(char **buf);

  void send_command(const std::string &command) const;
  void send_command(const char *command) const;
  bool read_back(char *buf, size_t &buflen);
  bool read_back();

  static void next_token(char **buf, uint8_t &val);
  static void next_token(char **buf, uint16_t &val);
  static void next_token(char **buf, uint32_t &val);
  static void next_token(char **buf, int32_t &val);
  static void next_token(char **buf, float &val);
  static void next_token(char **buf, std::string &val);
  static void next_token(char **buf);

private:
  bool connected_;
  LMSBuffer *buffer_;
  int socket_fd_;
};

using LMS1xxx = CoLaA; // The base implementation is for LMS1xxx

#endif // COLAA_H
