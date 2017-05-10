#ifndef COLAA_H
#define COLAA_H

#include <string>
#include <stdint.h>
#include <vector>

#include "LMS1xx/colaa_structs.h"

class LMSBuffer;

/**
 * @brief Abstraction for communication with SICK sensors that use
 * CoLa A ASCII Telegram messages.
 *
 * This class implements the protocol for the LMS1xx series of sensors as a baseline.
 *
 * Inherit from this class to implement new sensors that require custom
 * data formats.
 */
class CoLaA
{
public:
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
  void set_scan_config(const ScanConfig &cfg);

  /*!
  * @brief Set scan data configuration.
  * Set format of scan message returned by device.
  * @param cfg structure containing scan data configuration.
  */
  void set_scan_data_config(const ScanDataConfig &cfg);

  /*!
  * @brief Get current scan configuration.
  * Get scan configuration :
  * - scanning frequency.
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * @returns scanCfg structure.
  */
  ScanConfig get_scan_config();

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
  CoLaAStatus::Status query_status();

  /*!
  * @brief Get current output range configuration.
  * Get output range configuration :
  * - scanning resolution.
  * - start angle.
  * - stop angle.
  * Should be the same for all sensor types
  * @returns scanOutputRange structure.
  */
  ScanOutputRange get_scan_output_range();

  /*!
  * @brief Start or stop continuous data acquisition.
  * After reception of this command device start or stop continuous data stream containing scan messages.
  * @param start true : start false : stop
  */
  void scan_continuous(bool start);

  /*!
  * @brief Receive single scan message.
  * @param scan_data will be passed to parse_scan_data which can be overwritten by subclasses
  * @return true if scan was read successfully, false if error or timeout. False implies that higher level
  *         logic should take correct action such as reopening the connection.
  */
  bool get_scan_data(void *scan_data);

protected:
  // Command names
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

  /**
   * @brief Sends login command
   * @param user_class pick one of LOGIN_USER_x
   * @param password pick matching LOGIN_PASS_x
   */
  void do_login(std::string user_class, std::string password);

  /**
   * @brief Produce ScanConfig struct from raw ASCII message buffer
   * Can be overwritten by subclasses if customisation is needed.
   * @param buf
   * @param len
   * @return The parsed struct
   */
  virtual ScanConfig parse_scan_cfg(char *buf, size_t len);

  /**
   * @brief Build up the scan config string from a ScanConfig struct
   * Can be overwritten by subclasses if customisation is needed.
   * @param cfg
   * @return message string
   */
  virtual std::string build_scan_cfg(const ScanConfig &cfg) const;

  /**
   * @brief Build scan data config from struct
   * Base implementation will call build_scan_data_cfg_output_channel to construct the
   * output channel part of the message and build_scan_data_cfg_encoder to build the
   * encoder part of the message.
   * @param cfg
   * @return the message string
   */
  virtual std::string build_scan_data_cfg(const ScanDataConfig &cfg) const;

  /**
   * @brief Can be customised to implement special message formats for the output channel
   * @param ch Channel number as passed by the ScanDataConfig struct
   * @return output channel part of the message, should not contain leading/trailing spaces
   */
  virtual std::string build_scan_data_cfg_output_channel(int ch) const;

  /**
   * @brief Can be customised to implement special message formats for the encoder
   * @param enc Encoder setting as passed by the ScanDataConfig struct
   * @return encoder part of the message, should not include leading/trailing spaces
   */
  virtual std::string build_scan_data_cfg_encoder(int enc) const;

  /**
   * @brief Called by get_scan_data when the internal buffer is filled
   * @param buffer the message to be parsed
   * @param data Destination for the parsed data, pass a ScanData pointer for the base implementation
   */
  virtual void parse_scan_data(char *buffer, void *data) const;

  /**
   * @brief Parses the header part of the scan data message and returns it as a struct
   * @param buf the message data
   * @return filled struct
   */
  virtual ScanDataHeader parse_scan_data_header(char **buf) const;

  /**
   * @brief Parses the encoder part of the scan data message
   * The data is currently discarded.
   * @param buf the message data
   */
  virtual void parse_scan_data_encoderdata(char **buf) const;

  /**
   * @brief Surrounds command with start and end markers and sends them to the scanner
   * @param command The command string
   */
  void send_command(const std::string &command) const;

  /**
   * @brief Surrounds command with start and end markers and sends them to the scanner
   * @param command The command string
   */
  void send_command(const char *command) const;

  /**
   * @brief Read data back from the socket
   * Checks for malformed messages and parses error codes.
   * @param buf Destination for the read data
   * @param buflen Maximum size of the buffer. Will be set to new size of the buffer after the read.
   * @return True if the read was successful and no errors occured.
   */
  bool read_back(char *buf, size_t &buflen);

  /**
   * @brief Read data back from the socket and discard it
   * Error checks are still performed.
   * @return
   */
  bool read_back();

private:
  bool connected_;
  LMSBuffer *buffer_;
  int socket_fd_;
};

using LMS1xx = CoLaA; // CoLaA implements the protocol based on the LMS1xx sensor

#endif // COLAA_H
